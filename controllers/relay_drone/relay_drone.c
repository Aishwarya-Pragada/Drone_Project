/*
 * Crazyflie Relay Drone Controller
 *
 * Role: communication hub + fire verifier.
 *   - Waits (hovering outside the forest, where GPS is available) for alerts.
 *   - SMOKE alert  -> fly to the reported position -> scan -> centre the smoke in the camera
 *                     -> approach and descend -> confirm fire -> e-mail alert.
 *                     If no fire is seen (or the smoke cannot be found) the alarm is cleared
 *                     and the relay returns to its standby position.
 *   - FIRE alert   -> e-mail immediately (already confirmed by the exploration drone).
 *
 * POSITION: same scheme as the exploration drones.
 *   GPS is only available outside the forest (canopy mask in gps_available()).  Inside the
 *   canopy the position is dead-reckoned from a body-frame velocity sensor (simulated
 *   optical-flow deck) rotated by the IMU heading.  The e-mailed position and its
 *   uncertainty (sigma) come from this estimate.  Ground truth is used for logging and for
 *   the canopy mask only.
 *
 * ALERT FORMAT (from exp_drone.c):  "SMOKE x y sigma id"   /   "FIRE x y sigma id"
 *
 * Coordinate frame: Webots ENU, yaw 0 = +x (East), CCW positive.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <webots/camera.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/receiver.h>

#include "pid_controller.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---------------- Mission parameters ---------------- */
#define FLYING_ALTITUDE   1.0
#define DESCEND_ALTITUDE  0.5
#define FOREST_HALF       10.0
#define CANOPY_MARGIN     0.5
#define CRUISE_SPEED      0.6
#define MIN_SPEED         0.15
#define APPROACH_SPEED    0.3
#define ARRIVE_DIST       0.8    /* m                                                  */
#define KP_HEADING        1.5
#define MAX_YAW_RATE      1.0
#define SCAN_YAW_RATE     0.3    /* rad/s                                              */
#define ALIGN_TOL_PX      10
#define KP_ALIGN          0.004  /* rad/s per pixel while approaching                  */
#define MAX_ACCEL         0.5    /* m/s^2 slew limit on the forward-velocity setpoint  */
#define SETTLE_TIME       1.0    /* s                                                  */
#define NAV_TIMEOUT       150.0  /* s                                                  */
#define SCAN_TIMEOUT      50.0   /* s, about 2.4 revolutions                           */
#define APPROACH_MAX      8.0    /* m travelled towards the smoke before giving up     */
#define SMOKE_LOCK_PIXELS 80
#define FIRE_CONFIRM_PIXELS 50

/* ---------------- Sensor error model (must match exp_drone.c) ---------------- */
#define GPS_SIGMA         0.10
#define FLOW_SCALE_ERR    1.02
#define FLOW_BIAS         0.01
#define FLOW_NOISE        0.02   /* estimator only: white noise on the PID input destabilises it */
#define YAW_DRIFT_RATE    0.0002
#define DRIFT_K           0.05   /* position std growth per metre without GPS (measured 4-4.8 %) */

typedef enum { ST_TAKEOFF, ST_STANDBY, ST_NAVIGATE, ST_SCAN, ST_APPROACH, ST_CONFIRMED } state_t;

static double wrap_pi(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

static double clampd(double v, double lo, double hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static double randn(void) {
  double u1 = (rand() + 1.0) / (RAND_MAX + 2.0);
  double u2 = (rand() + 1.0) / (RAND_MAX + 2.0);
  return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

static int gps_available(double x, double y) {
  double lim = FOREST_HALF + CANOPY_MARGIN;
  return (fabs(x) > lim) || (fabs(y) > lim);
}

static void send_email(const char *who, double x, double y, double sigma) {
  char cmd[256];
  sprintf(cmd, "python send_alert.py %s %.2f %.2f %.2f", who, x, y, sigma);
  int rc = system(cmd);
  (void)rc;
}

int main(int argc, char **argv) {
  wb_robot_init();
  srand(4321);
  const int timestep = (int)wb_robot_get_basic_time_step();

  FILE *logfile = fopen("relay_drone_log.csv", "w");
  fprintf(logfile, "time,x,y,est_x,est_y,err,sigma,gps_ok,state\n"); /* x,y = ground truth (log only) */

  /* Motors */
  WbDeviceTag m1_motor = wb_robot_get_device("m1_motor");
  WbDeviceTag m2_motor = wb_robot_get_device("m2_motor");
  WbDeviceTag m3_motor = wb_robot_get_device("m3_motor");
  WbDeviceTag m4_motor = wb_robot_get_device("m4_motor");
  wb_motor_set_position(m1_motor, INFINITY);
  wb_motor_set_position(m2_motor, INFINITY);
  wb_motor_set_position(m3_motor, INFINITY);
  wb_motor_set_position(m4_motor, INFINITY);
  wb_motor_set_velocity(m1_motor, -1.0);
  wb_motor_set_velocity(m2_motor, 1.0);
  wb_motor_set_velocity(m3_motor, -1.0);
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
  WbDeviceTag receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, timestep);

  /* PID (Webots Crazyflie controller, unchanged gains) */
  actual_state_t actual_state = {0};
  desired_state_t desired_state = {0};
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
  motor_power_t motor_power;

  /* Stabilization delay */
  while (wb_robot_step(timestep) != -1) {
    if (wb_robot_get_time() > 2.0) break;
  }

  /* Estimator initialisation: the relay starts outside the forest, GPS is available */
  double est_x = wb_gps_get_values(gps)[0];
  double est_y = wb_gps_get_values(gps)[1];
  double home_x = est_x, home_y = est_y;
  double dist_since_fix = 0.0, sigma = GPS_SIGMA;
  double t_start = wb_robot_get_time();
  double past_time = t_start;

  state_t state = ST_TAKEOFF;
  double settle_t = 0.0, fwd_cmd = 0.0, state_t0 = t_start;
  double target_x = 0, target_y = 0, target_sigma = 0;
  int target_is_home = 0;
  double approach_x0 = 0, approach_y0 = 0;
  int alert_source = -1;
  double last_smoke_cx = -1;
  double side_cmd = 0.0, hold_fx = est_x, hold_fy = est_y;

  printf("RELAY DRONE: taking off, then waiting for alerts (home %.2f, %.2f)\n", home_x, home_y);

  while (wb_robot_step(timestep) != -1) {
    double t = wb_robot_get_time();
    double dt = t - past_time;
    if (dt < 0.0001) dt = 0.0001;

    /* ---- raw sensors ---- */
    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);
    double yaw_true = rpy[2];
    double yaw_meas = wrap_pi(yaw_true + YAW_DRIFT_RATE * (t - t_start));
    actual_state.roll = rpy[0];
    actual_state.pitch = rpy[1];
    actual_state.yaw_rate = wb_gyro_get_values(gyro)[2];
    const double *gp = wb_gps_get_values(gps);
    actual_state.altitude = gp[2]; /* simulated ToF height sensor */
    double true_x = gp[0], true_y = gp[1]; /* GROUND TRUTH: logging + canopy mask only */

    /* simulated flow deck (see exp_drone.c) */
    const double *sv = wb_gps_get_speed_vector(gps);
    double vbx = sv[0] * cos(yaw_true) + sv[1] * sin(yaw_true);
    double vby = -sv[0] * sin(yaw_true) + sv[1] * cos(yaw_true);
    double vbx_pid = FLOW_SCALE_ERR * vbx + FLOW_BIAS;
    double vby_pid = FLOW_SCALE_ERR * vby + FLOW_BIAS;
    double vbx_m = vbx_pid + FLOW_NOISE * randn();
    double vby_m = vby_pid + FLOW_NOISE * randn();
    actual_state.vx = vbx_pid;
    actual_state.vy = vby_pid;

    /* ---- position estimator ---- */
    int gps_ok = gps_available(true_x, true_y);
    if (gps_ok) {
      est_x = true_x + GPS_SIGMA * randn();
      est_y = true_y + GPS_SIGMA * randn();
      dist_since_fix = 0.0;
    } else {
      double vgx = vbx_m * cos(yaw_meas) - vby_m * sin(yaw_meas);
      double vgy = vbx_m * sin(yaw_meas) + vby_m * cos(yaw_meas);
      est_x += vgx * dt;
      est_y += vgy * dt;
      dist_since_fix += sqrt(vgx * vgx + vgy * vgy) * dt;
    }
    /* uncertainty: GPS noise + scale/bias drift + un-synchronised heading drift (relay never re-syncs heading) */
    sigma = sqrt(GPS_SIGMA * GPS_SIGMA + pow(DRIFT_K * dist_since_fix, 2) +
                 pow(YAW_DRIFT_RATE * (t - t_start) * dist_since_fix, 2));
    double err = sqrt(pow(est_x - true_x, 2) + pow(est_y - true_y, 2));
    fprintf(logfile, "%f,%f,%f,%f,%f,%f,%f,%d,%d\n", t, true_x, true_y, est_x, est_y, err, sigma, gps_ok, (int)state);
    fflush(logfile);

    /* ---- receive alerts ---- */
    while (wb_receiver_get_queue_length(receiver) > 0) {
      const char *msg = (const char *)wb_receiver_get_data(receiver);
      double mx, my, ms = 0.0;
      int mid = -1;

      int n = sscanf(msg, "SMOKE %lf %lf %lf %d", &mx, &my, &ms, &mid);
      if (n >= 2) {
        if (state == ST_STANDBY) {
          target_x = mx;
          target_y = my;
          target_sigma = (n >= 3) ? ms : 0.0;
          alert_source = (n >= 4) ? mid : -1;
          target_is_home = 0;
          state = ST_NAVIGATE;
          state_t0 = t;
          printf("\nRELAY DRONE: SMOKE ALERT from drone %d -> target (%.2f, %.2f) +-%.2f\n", alert_source, mx, my, target_sigma);
        } else {
          printf("RELAY DRONE: smoke alert ignored (busy, state %d)\n", (int)state);
        }
      } else {
        n = sscanf(msg, "FIRE %lf %lf %lf %d", &mx, &my, &ms, &mid);
        if (n >= 2) {
          char who[48];
          sprintf(who, "Exploration_Drone_%d", (n >= 4) ? mid : 0);
          printf("\nRELAY DRONE: FIRE ALERT from %s at (%.2f, %.2f) +-%.2f -> sending e-mail\n", who, mx, my, (n >= 3) ? ms : 0.0);
          send_email(who, mx, my, (n >= 3) ? ms : 0.0);
        }
      }
      wb_receiver_next_packet(receiver);
    }

    /* ---- camera analysis ---- */
    const unsigned char *image = wb_camera_get_image(camera);
    int smoke_pixels = 0, fire_pixels = 0, smoke_x_sum = 0, smoke_count = 0;
    for (int ix = 0; ix < width; ix += 4) {
      for (int iy = 0; iy < height; iy += 4) {
        int r = wb_camera_image_get_red(image, width, ix, iy);
        int g = wb_camera_image_get_green(image, width, ix, iy);
        int b = wb_camera_image_get_blue(image, width, ix, iy);
        if (abs(r - g) < 20 && abs(r - b) < 20 && abs(g - b) < 20 && r > 120) {
          smoke_pixels++;
          smoke_x_sum += ix;
          smoke_count++;
        }
        if (r > 200 && g > 100 && b < 100) fire_pixels++;
      }
    }
    double smoke_center_x = (smoke_count > 0) ? (double)smoke_x_sum / smoke_count : -1;
    /* ---- end camera analysis ---- */

    /* ---- state machine ---- */
    double height_desired = FLYING_ALTITUDE;
    double forward_desired = 0.0, yaw_rate_cmd = 0.0, hold_fwd = 0.0, hold_side = 0.0;

    switch (state) {
      case ST_TAKEOFF:
        if (fabs(actual_state.altitude - height_desired) < 0.1) settle_t += dt; else settle_t = 0.0;
        if (settle_t >= SETTLE_TIME) {
          state = ST_STANDBY;
          printf("RELAY DRONE: Waiting for alerts\n");
        }
        break;

      case ST_STANDBY: { /* hold the standby position (GPS is available outside the forest) */
        /* smooth the noisy GPS fix and use a deadband: noise on a setpoint is amplified by the
           PID's derivative term */
        hold_fx += 0.05 * (est_x - hold_fx);
        hold_fy += 0.05 * (est_y - hold_fy);
        double ex = home_x - hold_fx, ey = home_y - hold_fy;
        if (sqrt(ex * ex + ey * ey) > 0.3) {
          hold_fwd = clampd(0.2 * (ex * cos(yaw_meas) + ey * sin(yaw_meas)), -0.3, 0.3);
          hold_side = clampd(0.2 * (-ex * sin(yaw_meas) + ey * cos(yaw_meas)), -0.3, 0.3);
        }
        break;
      }

      case ST_NAVIGATE: { /* turn towards the target, then fly nose-first to it */
        double dx = target_x - est_x, dy = target_y - est_y;
        double dist = sqrt(dx * dx + dy * dy);
        double herr = wrap_pi(atan2(dy, dx) - yaw_meas);
        if (dist < ARRIVE_DIST) {
          if (target_is_home) {
            state = ST_STANDBY;
            printf("RELAY DRONE: back at standby position. Waiting for alerts\n");
          } else {
            state = ST_SCAN;
            printf("RELAY DRONE: reached reported position. Scanning...\n");
          }
          state_t0 = t;
        } else if (t - state_t0 > NAV_TIMEOUT) {
          printf("RELAY DRONE: navigation timeout, returning to standby position\n");
          target_x = home_x; target_y = home_y; target_is_home = 1; state_t0 = t;
        } else {
          yaw_rate_cmd = KP_HEADING * herr;
          if (fabs(herr) < 0.35) forward_desired = clampd(0.3 * dist, MIN_SPEED, CRUISE_SPEED);
        }
        break;
      }

      case ST_SCAN:
        if (smoke_pixels > SMOKE_LOCK_PIXELS) {
          double error = smoke_center_x - width / 2.0;
          if (fabs(error) > ALIGN_TOL_PX) {
            yaw_rate_cmd = (error > 0) ? -SCAN_YAW_RATE : SCAN_YAW_RATE; /* smoke on the right -> turn CW */
          } else {
            state = ST_APPROACH;
            approach_x0 = est_x; approach_y0 = est_y; state_t0 = t;
            printf("RELAY DRONE: smoke centred. Moving towards smoke\n");
          }
        } else {
          yaw_rate_cmd = SCAN_YAW_RATE;
        }
        if (state == ST_SCAN && t - state_t0 > SCAN_TIMEOUT) {
          printf("RELAY DRONE: smoke not found -> alarm cleared, returning to standby\n");
          target_x = home_x; target_y = home_y; target_is_home = 1; state = ST_NAVIGATE; state_t0 = t;
        }
        break;

      case ST_APPROACH: {
        height_desired = DESCEND_ALTITUDE;
        forward_desired = APPROACH_SPEED;
        if (smoke_count > 20) {
          last_smoke_cx = smoke_center_x;
          yaw_rate_cmd = clampd(-KP_ALIGN * (smoke_center_x - width / 2.0), -SCAN_YAW_RATE, SCAN_YAW_RATE);
        }
        double travelled = sqrt(pow(est_x - approach_x0, 2) + pow(est_y - approach_y0, 2));
        if (fire_pixels > FIRE_CONFIRM_PIXELS) {
          printf("RELAY DRONE: FIRE CONFIRMED at est (%.2f, %.2f) +-%.2f\n", est_x, est_y, sigma);
          send_email("Relay_Drone", est_x, est_y, sigma);
          state = ST_CONFIRMED;
        } else if (travelled > APPROACH_MAX) {
          printf("RELAY DRONE: no fire found near the smoke -> FALSE ALARM cleared, returning to standby\n");
          target_x = home_x; target_y = home_y; target_is_home = 1; state = ST_NAVIGATE; state_t0 = t;
        }
        break;
      }

      case ST_CONFIRMED:
        height_desired = DESCEND_ALTITUDE; /* hover over the fire, alert already sent */
        break;
    }
    (void)last_smoke_cx; (void)target_sigma;

    /* ---- setpoints: slew-limit both velocity axes, always fly nose-first ---- */
    {
      double dv = MAX_ACCEL * dt;
      double fw = forward_desired + hold_fwd;
      if (fw > fwd_cmd + dv) fwd_cmd += dv;
      else if (fw < fwd_cmd - dv) fwd_cmd -= dv;
      else fwd_cmd = fw;
      if (hold_side > side_cmd + dv) side_cmd += dv;
      else if (hold_side < side_cmd - dv) side_cmd -= dv;
      else side_cmd = hold_side;
    }
    desired_state.roll = 0;
    desired_state.pitch = 0;
    desired_state.vx = fwd_cmd;
    desired_state.vy = side_cmd;
    desired_state.yaw_rate = clampd(yaw_rate_cmd, -MAX_YAW_RATE, MAX_YAW_RATE);
    desired_state.altitude = height_desired;

    pid_velocity_fixed_height_controller(actual_state, &desired_state, gains_pid, dt, &motor_power);

    if (!isnan(motor_power.m1) && !isnan(motor_power.m2) && !isnan(motor_power.m3) && !isnan(motor_power.m4)) {
      wb_motor_set_velocity(m1_motor, -motor_power.m1);
      wb_motor_set_velocity(m2_motor, motor_power.m2);
      wb_motor_set_velocity(m3_motor, -motor_power.m3);
      wb_motor_set_velocity(m4_motor, motor_power.m4);
    }

    past_time = t;
  }

  fclose(logfile);
  wb_robot_cleanup();
  return 0;
}