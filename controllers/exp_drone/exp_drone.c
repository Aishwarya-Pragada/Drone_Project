/*
 * Crazyflie Exploration Drone Controller (command-based patrol + dead reckoning)
 *
 * controllerArgs in the .wbt:  "0"  -> horizontal strips, start (-11.3, 9), heading East
 *                              "1"  -> vertical strips,   start (-9, 11.3), heading South
 *
 * Coordinate frame: Webots ENU, world centre (0,0), yaw 0 = +x (East), CCW positive.
 * World: 25 x 25 m (edges +-12.5).  Forest: 20 x 20 m (edges +-10).
 *
 * KEY IDEAS
 *  1. The mission is a list of commands: FORWARD(distance) and TURN(absolute heading).
 *     There are no waypoints.  The drone always flies nose-first; it yaws to change direction.
 *  2. The position estimate (x_est, y_est) is what the controller uses for everything.
 *     - GPS is only *available* outside the forest (canopy mask, see gps_available()).
 *     - Inside the forest the estimate is dead-reckoned from a body-frame velocity
 *       sensor (simulated optical-flow deck) rotated by the IMU heading.
 *  3. Ground truth (true_x, true_y) is written to the log ONLY, for error analysis.
 *
 * NOTE: the flow-deck model reads the simulator's speed vector and adds scale error,
 * bias and noise.  State this in the paper as a simulated sensor model.
 *
 * STABILITY NOTE: the Webots PID differentiates the velocity error (kd_vel_xy) over one
 * 32 ms step, so white noise on the velocity fed to the PID destabilises the drone.
 * White noise is therefore applied ONLY to the estimator's copy of the velocity; the
 * PID receives the bias/scale-corrupted but noise-free velocity (as if from an onboard
 * EKF).  Setpoints are slew-limited so the PID never sees a step.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/emitter.h>

#include "pid_controller.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---------------- Mission / environment parameters ---------------- */
#define FOREST_HALF      10.0   /* forest edge (+-10 m)                          */
#define CANOPY_MARGIN    0.5    /* GPS fix returns only this far outside forest  */
#define CRUISE_SPEED     0.6    /* m/s                                           */
#define MIN_SPEED        0.15   /* m/s, used for final approach of a leg         */
#define LEG_TOLERANCE    0.05   /* m                                             */
#define HEADING_TOL_DEG  3.0
#define KP_HEADING       1.5
#define MAX_YAW_RATE     1.0    /* rad/s                                         */
#define YAW_CMD_SIGN     1.0    /* +1: positive yaw_rate = CCW (verified with relay drone)  */
#define MAX_ACCEL        0.5    /* m/s^2 slew limit on the forward-velocity setpoint         */
#define SETTLE_TIME      1.0    /* s within altitude band before motion commands start       */
#define MIN_SYNC_LEG     8.0    /* m, min all-GPS leg length used for heading re-sync        */
#define FAILSAFE_OVERRUN 1.0    /* m, max extra distance while waiting for a GPS fix         */

/* ---------------- Sensor error model (for the drift study) --------- */
#define GPS_SIGMA        0.10   /* m, GPS position noise (outside forest)        */
#define FLOW_SCALE_ERR   1.02   /* 2 % velocity scale error                      */
#define FLOW_BIAS        0.01   /* m/s bias                                      */
#define FLOW_NOISE       0.02   /* m/s std, estimator only (see STABILITY NOTE)  */
#define YAW_DRIFT_RATE   0.0002 /* rad/s gyro/IMU heading drift                  */
#define DRIFT_K          0.05   /* position std growth per metre without GPS (measured 4-4.8 % in tests) */

#define FIRE_STOP_THRESHOLD 180
#define SMOKE_THRESHOLD     120

/* ---------------- Command definitions ------------------------------ */
typedef enum { CMD_FORWARD, CMD_TURN } cmd_type_t;
typedef struct {
  cmd_type_t type;
  double value; /* FORWARD: metres.  TURN: absolute heading in rad */
} command_t;

#define E   0.0
#define N   (M_PI / 2.0)
#define W   M_PI
#define S   (-M_PI / 2.0)

/* Turn lines are at +-11.3 m: outside the 10.5 m GPS band (forest edge 10 + margin) and
 * 1.2 m inside the 12.5 m arena wall.  Dead reckoning can run ~1 m short over a strip, so
 * every leg completes only after GPS is re-acquired (see the executor).
 *
 * Drone 0: start (-11.3, 9) heading E. Strips at y = 9, 0, -9, then return outside. */
static const command_t PROGRAM_HORIZONTAL[] = {
    {CMD_FORWARD, 22.6}, {CMD_TURN, S}, {CMD_FORWARD, 9},   /* (11.3,9)  -> (11.3,0)    */
    {CMD_TURN, W},       {CMD_FORWARD, 22.6},               /* -> (-11.3,0)              */
    {CMD_TURN, S},       {CMD_FORWARD, 9},                  /* -> (-11.3,-9)             */
    {CMD_TURN, E},       {CMD_FORWARD, 22.6},               /* -> (11.3,-9)              */
    {CMD_TURN, N},       {CMD_FORWARD, 20.3},               /* -> (11.3,11.3)  outside   */
    {CMD_TURN, W},       {CMD_FORWARD, 22.6},               /* -> (-11.3,11.3) outside   */
    {CMD_TURN, S},       {CMD_FORWARD, 2.3},                /* -> (-11.3,9)              */
    {CMD_TURN, E}};

/* Drone 1: start (-9, 11.3) heading S. Strips at x = -9, 0, 9, then return outside. */
static const command_t PROGRAM_VERTICAL[] = {
    {CMD_FORWARD, 22.6}, {CMD_TURN, E}, {CMD_FORWARD, 9},   /* (-9,-11.3) -> (0,-11.3)  */
    {CMD_TURN, N},       {CMD_FORWARD, 22.6},               /* -> (0,11.3)               */
    {CMD_TURN, E},       {CMD_FORWARD, 9},                  /* -> (9,11.3)               */
    {CMD_TURN, S},       {CMD_FORWARD, 22.6},               /* -> (9,-11.3)              */
    {CMD_TURN, W},       {CMD_FORWARD, 20.3},               /* -> (-11.3,-11.3) outside  */
    {CMD_TURN, N},       {CMD_FORWARD, 22.6},               /* -> (-11.3,11.3)  outside  */
    {CMD_TURN, E},       {CMD_FORWARD, 2.3},                /* -> (-9,11.3)              */
    {CMD_TURN, S}};

/* Known start pose of each drone (must match the .wbt).  The command list defines a planned
 * pose that advances exactly by each command, so every leg is measured against the plan. */
static const double START_X[2] = {-11.3, -9.0};
static const double START_Y[2] = {9.0, 11.3};

/* ---------------- Helpers ------------------------------------------ */
static double wrap_pi(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

static double clampd(double v, double lo, double hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static double randn(void) { /* Box-Muller */
  double u1 = (rand() + 1.0) / (RAND_MAX + 2.0);
  double u2 = (rand() + 1.0) / (RAND_MAX + 2.0);
  return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

/* Canopy model: GPS is unavailable inside the forest (with a small margin). */
static int gps_available(double x, double y) {
  double lim = FOREST_HALF + CANOPY_MARGIN;
  return (fabs(x) > lim) || (fabs(y) > lim);
}

int main(int argc, char **argv) {
  int drone_id = (argc > 1) ? atoi(argv[1]) : 0;

  const command_t *program = drone_id == 0 ? PROGRAM_HORIZONTAL : PROGRAM_VERTICAL;
  const int program_len = drone_id == 0 ? (int)(sizeof(PROGRAM_HORIZONTAL) / sizeof(command_t))
                                        : (int)(sizeof(PROGRAM_VERTICAL) / sizeof(command_t));
  const double height_desired = drone_id == 0 ? 1.2 : 1.8; /* 0.6 m vertical separation (relay hovers at 1.0) */
  double heading_ref = drone_id == 0 ? E : S;               /* must match .wbt yaw */

  wb_robot_init();
  srand(1234 + drone_id);
  const int timestep = (int)wb_robot_get_basic_time_step();

  char logname[64];
  sprintf(logname, "exp_drone%d_log.csv", drone_id + 1); /* matches drone_visualization.py */
  FILE *logfile = fopen(logname, "w");
  fprintf(logfile, "time,x,y,est_x,est_y,err,sigma,gps_ok,cmd_idx\n"); /* x,y = ground truth (log only) */

  /* Motors */
  WbDeviceTag m1_motor = wb_robot_get_device("m1_motor");
  wb_motor_set_position(m1_motor, INFINITY);
  wb_motor_set_velocity(m1_motor, -1.0);
  WbDeviceTag m2_motor = wb_robot_get_device("m2_motor");
  wb_motor_set_position(m2_motor, INFINITY);
  wb_motor_set_velocity(m2_motor, 1.0);
  WbDeviceTag m3_motor = wb_robot_get_device("m3_motor");
  wb_motor_set_position(m3_motor, INFINITY);
  wb_motor_set_velocity(m3_motor, -1.0);
  WbDeviceTag m4_motor = wb_robot_get_device("m4_motor");
  wb_motor_set_position(m4_motor, INFINITY);
  wb_motor_set_velocity(m4_motor, 1.0);

  /* Sensors */
  WbDeviceTag imu = wb_robot_get_device("inertial_unit");
  wb_inertial_unit_enable(imu, timestep);
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, timestep);
  WbDeviceTag gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, timestep);
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, timestep);
  int width = wb_camera_get_width(camera);
  int height = wb_camera_get_height(camera);
  WbDeviceTag emitter = wb_robot_get_device("emitter");

  /* Stabilization delay */
  while (wb_robot_step(timestep) != -1) {
    if (wb_robot_get_time() > 2.0) break;
  }

  actual_state_t actual_state = {0};
  desired_state_t desired_state = {0};
  motor_power_t motor_power;

  /* Same Webots PID gains as before (altitude PID, velocity PD, attitude PD) */
  gains_pid_t gains_pid;
  gains_pid.kp_att_y = 1;
  gains_pid.kd_att_y = 0.5;
  gains_pid.kp_att_rp = 0.5;
  gains_pid.kd_att_rp = 0.1;
  gains_pid.kp_vel_xy = 2;
  gains_pid.kd_vel_xy = 0.5;
  gains_pid.kp_z = 10;
  gains_pid.ki_z = 5;
  gains_pid.kd_z = 5;
  init_pid_attitude_fixed_height_controller();

  /* Estimator initialisation: drone starts outside the forest, GPS is available */
  double est_x = wb_gps_get_values(gps)[0];
  double est_y = wb_gps_get_values(gps)[1];
  double dist_since_fix = 0.0;
  double sigma = GPS_SIGMA;
  double t_start = wb_robot_get_time();
  double past_time = t_start;

  int cmd_idx = 0, leg_started = 0;
  double leg_x0 = 0, leg_y0 = 0;            /* estimate at leg start (for heading re-sync) */
  double plan_x = START_X[drone_id % 2];     /* planned pose: advances exactly by the commands */
  double plan_y = START_Y[drone_id % 2];
  int airborne = 0, event_hold = 0;
  double settle_t = 0.0, fwd_cmd = 0.0, yaw_bias = 0.0;
  int leg_all_gps = 1;
  int smoke_sent = 0, fire_sent = 0;

  printf("EXPLORATION DRONE %d: command patrol started\n", drone_id);

  while (wb_robot_step(timestep) != -1) {
    double t = wb_robot_get_time();
    double dt = t - past_time;
    if (dt < 0.0001) dt = 0.0001;

    /* ---- raw sensors ---- */
    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);
    double yaw_true = rpy[2];
    double yaw_raw = wrap_pi(yaw_true + YAW_DRIFT_RATE * (t - t_start)); /* drifting IMU heading */
    double yaw_meas = wrap_pi(yaw_raw - yaw_bias);                        /* after GPS re-sync    */

    actual_state.roll = rpy[0];
    actual_state.pitch = rpy[1];
    actual_state.yaw_rate = wb_gyro_get_values(gyro)[2];
    actual_state.altitude = wb_gps_get_values(gps)[2]; /* simulated ToF height sensor */

    const double *gp = wb_gps_get_values(gps);
    double true_x = gp[0], true_y = gp[1]; /* GROUND TRUTH: logging + canopy mask only */

    /* ---- simulated flow-deck: body-frame velocity with scale error, bias, noise ---- */
    const double *sv = wb_gps_get_speed_vector(gps);
    double vbx = sv[0] * cos(yaw_true) + sv[1] * sin(yaw_true);
    double vby = -sv[0] * sin(yaw_true) + sv[1] * cos(yaw_true);
    double vbx_pid = FLOW_SCALE_ERR * vbx + FLOW_BIAS; /* to the PID: no white noise */
    double vby_pid = FLOW_SCALE_ERR * vby + FLOW_BIAS;
    double vbx_m = vbx_pid + FLOW_NOISE * randn();      /* to the estimator only     */
    double vby_m = vby_pid + FLOW_NOISE * randn();
    actual_state.vx = vbx_pid; /* velocity feedback for the Webots PID */
    actual_state.vy = vby_pid;

    /* ---- position estimator ---- */
    int gps_ok = gps_available(true_x, true_y);
    if (gps_ok) {
      est_x = true_x + GPS_SIGMA * randn(); /* fix / re-sync */
      est_y = true_y + GPS_SIGMA * randn();
      dist_since_fix = 0.0;
    } else {
      double vgx = vbx_m * cos(yaw_meas) - vby_m * sin(yaw_meas);
      double vgy = vbx_m * sin(yaw_meas) + vby_m * cos(yaw_meas);
      est_x += vgx * dt;
      est_y += vgy * dt;
      dist_since_fix += sqrt(vgx * vgx + vgy * vgy) * dt;
    }
    sigma = sqrt(GPS_SIGMA * GPS_SIGMA + pow(DRIFT_K * dist_since_fix, 2));

    double err = sqrt(pow(est_x - true_x, 2) + pow(est_y - true_y, 2));
    fprintf(logfile, "%f,%f,%f,%f,%f,%f,%f,%d,%d\n", t, true_x, true_y, est_x, est_y, err, sigma, gps_ok, cmd_idx);
    fflush(logfile);

    /* ---- vision: smoke / fire ---- */
    const unsigned char *image = wb_camera_get_image(camera);
    int smoke_pixels = 0, fire_pixels = 0;
    for (int x = 0; x < width; x += 4) {
      for (int y = 0; y < height; y += 4) {
        int r = wb_camera_image_get_red(image, width, x, y);
        int g = wb_camera_image_get_green(image, width, x, y);
        int b = wb_camera_image_get_blue(image, width, x, y);
        if (r > 200 && g > 100 && b < 100) fire_pixels++;
        if (abs(r - g) < 20 && abs(r - b) < 20 && abs(g - b) < 20 && r > 120 && r < 230) smoke_pixels++;
      }
    }

    /* Message format: TYPE est_x est_y sigma drone_id  (first two numbers match the old
       'TYPE x y' format, so relay_drone.c's sscanf keeps working unchanged) */
    if (airborne && fire_pixels > FIRE_STOP_THRESHOLD && !fire_sent) {
      char msg[96];
      sprintf(msg, "FIRE %.2f %.2f %.2f %d", est_x, est_y, sigma, drone_id);
      wb_emitter_send(emitter, msg, strlen(msg) + 1);
      printf("DRONE %d: FIRE at est (%.2f, %.2f) +-%.2f\n", drone_id, est_x, est_y, sigma);
      fire_sent = 1;
      event_hold = 1;
    } else if (airborne && smoke_pixels > SMOKE_THRESHOLD && !smoke_sent && !fire_sent) {
      char msg[96];
      sprintf(msg, "SMOKE %.2f %.2f %.2f %d", est_x, est_y, sigma, drone_id);
      wb_emitter_send(emitter, msg, strlen(msg) + 1);
      printf("DRONE %d: SMOKE at est (%.2f, %.2f) +-%.2f\n", drone_id, est_x, est_y, sigma);
      smoke_sent = 1;
      event_hold = 1;
    }

    /* ---- command executor ---- */
    double forward_desired = 0.0, yaw_rate_cmd = 0.0;

    if (!airborne) { /* take-off: no motion commands until altitude has settled */
      if (fabs(actual_state.altitude - height_desired) < 0.1) settle_t += dt; else settle_t = 0.0;
      if (settle_t >= SETTLE_TIME) airborne = 1;
    } else if (!event_hold) {
      const command_t *c = &program[cmd_idx];

      if (c->type == CMD_FORWARD) {
        if (!leg_started) {
          leg_x0 = est_x;
          leg_y0 = est_y;
          leg_started = 1;
          leg_all_gps = 1;
        }
        if (!gps_ok) leg_all_gps = 0;
        /* Along-track progress is measured from the PLANNED leg start, so any error left
           over from the previous leg is absorbed here instead of accumulating. */
        double along = (est_x - plan_x) * cos(heading_ref) + (est_y - plan_y) * sin(heading_ref);
        double remaining = c->value - along;
        /* Every leg ends outside the forest, so a leg only completes once GPS has been
           re-acquired (the estimate is then re-synced to truth).  If dead reckoning ran
           short, the drone keeps creeping forward until it exits the canopy.  The overrun
           cap protects against a lost GPS fix (the arena wall is only 1.5 m past the turn). */
        int arrived = (gps_ok && remaining <= LEG_TOLERANCE) || (remaining <= -FAILSAFE_OVERRUN);
        if (arrived) {
          /* Heading re-sync: on a long leg flown entirely with GPS, the GPS course is a
             good heading reference, so the accumulated IMU heading drift can be removed. */
          double path = sqrt(pow(est_x - leg_x0, 2) + pow(est_y - leg_y0, 2));
          if (leg_all_gps && path >= MIN_SYNC_LEG) {
            double course = atan2(est_y - leg_y0, est_x - leg_x0);
            yaw_bias = wrap_pi(yaw_raw - course);
          }
          plan_x += c->value * cos(heading_ref); /* advance the planned pose by the command */
          plan_y += c->value * sin(heading_ref);
          cmd_idx = (cmd_idx + 1) % program_len;
          leg_started = 0;
        } else {
          forward_desired = clampd(0.8 * remaining, MIN_SPEED, CRUISE_SPEED);
          yaw_rate_cmd = KP_HEADING * wrap_pi(heading_ref - yaw_meas); /* hold heading */
        }
      } else { /* CMD_TURN: rotate in place to an absolute heading */
        heading_ref = c->value;
        double herr = wrap_pi(heading_ref - yaw_meas);
        if (fabs(herr) < HEADING_TOL_DEG * M_PI / 180.0 && fabs(actual_state.yaw_rate) < 0.1) {
          cmd_idx = (cmd_idx + 1) % program_len;
        } else {
          yaw_rate_cmd = KP_HEADING * herr;
        }
      }
      yaw_rate_cmd = clampd(yaw_rate_cmd, -MAX_YAW_RATE, MAX_YAW_RATE);
    }

    desired_state.roll = 0;
    desired_state.pitch = 0;
    { /* slew-limit the forward setpoint: the PID differentiates the velocity error */
      double dv = MAX_ACCEL * dt;
      if (forward_desired > fwd_cmd + dv) fwd_cmd += dv;
      else if (forward_desired < fwd_cmd - dv) fwd_cmd -= dv;
      else fwd_cmd = forward_desired;
    }
    desired_state.vx = fwd_cmd; /* always forward, never sideways */
    desired_state.vy = 0.0;
    desired_state.yaw_rate = YAW_CMD_SIGN * yaw_rate_cmd;
    desired_state.altitude = height_desired;

    pid_velocity_fixed_height_controller(actual_state, &desired_state, gains_pid, dt, &motor_power);

    wb_motor_set_velocity(m1_motor, -motor_power.m1);
    wb_motor_set_velocity(m2_motor, motor_power.m2);
    wb_motor_set_velocity(m3_motor, -motor_power.m3);
    wb_motor_set_velocity(m4_motor, motor_power.m4);

    past_time = t;
  }

  fclose(logfile);
  wb_robot_cleanup();
  return 0;
}