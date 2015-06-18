# Drone Pilot

Automatic pilot software that can control and fly several drones, including Pixhawk's, APM's and MultiWii's

## Current scripts:

* mw-joystick-test.py -> Send joystick commands via UDP from a ground-station running Matlab to a FC running MultiWii software.

* mw-hover-controller.py -> Calculate commands to make a Multiwii multicopter hover over a specified x,y,z coordinate.

* pix-logdata.py -> Script that logs data from a computer vision thread and from a flying multicopter with a Pixhawk. DroneApi related.

* pix-takeoff.py -> Script that makes a pixhawk take off in the most secure way. DroneApi related.

## Caution

This code is still under heavy development, everyday I add and remove stuff. Proceed with caution.

## Social networks:

You can follow us in this URL's:

* [Aldux.net](http://aldux.net/)
* [Aldux.net Facebook](https://www.facebook.com/AlduxNet)

## Assumptions:

For the 'pix-' scripts the following assumptions must be noted:

* This script assumes that the pixhawk is connected to the raspeberry pi via the serial port (/dev/ttyAMA0)
* In our setup, telemetry port 2 is configured at 115200 on the pixhawk
* rpi camera connected (for computer vision) 