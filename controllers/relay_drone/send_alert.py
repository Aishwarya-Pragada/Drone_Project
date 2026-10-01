import smtplib
import sys
import math
from email.message import EmailMessage


# ---------------------------------------------------------------------------
# Geo-reference of the simulated world.
#
# The Webots arena centre (0, 0) is mapped to (LAT0, LON0).
# x = East (metres)
# y = North (metres)
#
# Replace these values with the real site if required.
# ---------------------------------------------------------------------------

LAT0 = 40.0150
LON0 = -105.2705

METRES_PER_DEG_LAT = 111320.0


def local_to_latlon(x, y):
    """
    Convert local Webots coordinates in metres to
    latitude/longitude.

    x = East
    y = North
    """

    lat = LAT0 + y / METRES_PER_DEG_LAT

    lon = LON0 + x / (
        METRES_PER_DEG_LAT * math.cos(math.radians(LAT0))
    )

    return lat, lon


def send_alert(drone_name, x, y, sigma):

    # -----------------------------------------------------------------------
    # Convert Webots position to latitude/longitude
    # -----------------------------------------------------------------------

    latitude, longitude = local_to_latlon(x, y)

    # -----------------------------------------------------------------------
    # Google Maps link
    # -----------------------------------------------------------------------

    maps_link = (
        f"https://www.google.com/maps"
        f"?q={latitude:.6f},{longitude:.6f}"
    )

    # -----------------------------------------------------------------------
    # Email body
    # -----------------------------------------------------------------------

    body = f"""
*** EMERGENCY ALERT ***

Fire has been detected by: {drone_name}

Location Details
----------------

Local position (m):
x = {x:.2f} (East)
y = {y:.2f} (North)

Latitude  : {latitude:.6f}
Longitude : {longitude:.6f}

Position uncertainty:
+/- {sigma:.2f} m (dead-reckoned estimate)

Note:
This is the position of the reporting drone,
which is within a few metres of the fire.

View location on Google Maps:
{maps_link}

Immediate action required.
"""

    # -----------------------------------------------------------------------
    # Print alert to Webots/terminal
    # -----------------------------------------------------------------------

    print(body)

    # -----------------------------------------------------------------------
    # Create email
    # -----------------------------------------------------------------------

    msg = EmailMessage()

    msg.set_content(body)

    msg["Subject"] = "DRONE SWARM FIRE ALERT"

    sender_email = "sender@email.com"
    target_email = "receiver@email.com"

    msg["From"] = sender_email
    msg["To"] = target_email

    # -----------------------------------------------------------------------
    # Send email through Gmail SMTP
    # -----------------------------------------------------------------------

    try:

        server = smtplib.SMTP_SSL(
            "smtp.gmail.com",
            465
        )

        # IMPORTANT:
        # Use a Gmail APP PASSWORD here.
        server.login(
            sender_email,
            "password"
        )

        server.send_message(msg)

        server.quit()

        print(
            f"Email alert sent from {drone_name}!"
        )

    except Exception as e:

        print(
            f"Failed to send email: {e}"
        )


# ---------------------------------------------------------------------------
# Main
#
# Usage:
# python send_alert.py <drone_name> <x_m> <y_m> [sigma_m]
#
# Example:
# python send_alert.py Exploration_Drone_1 -0.22 -11.27 0.10
# ---------------------------------------------------------------------------

if __name__ == "__main__":

    drone = (
        sys.argv[1]
        if len(sys.argv) > 1
        else "Unknown Drone"
    )

    x = (
        float(sys.argv[2])
        if len(sys.argv) > 2
        else 0.0
    )

    y = (
        float(sys.argv[3])
        if len(sys.argv) > 3
        else 0.0
    )

    sigma = (
        float(sys.argv[4])
        if len(sys.argv) > 4
        else 0.0
    )

    send_alert(
        drone,
        x,
        y,
        sigma
    )
