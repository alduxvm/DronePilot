# Drone Pilot

 Pilot software (running on companion computers) for the flight controller autopilot's. It can control and fly several flight controllers, including Pixhawk's, APM's and MultiWii's.

## Current scripts:

* mw-joystick-test.py -> Send joystick commands via UDP from a ground-station running Matlab to a FC running MultiWii software.

* mw-hover-controller.py -> Calculate commands to make a Multiwii multicopter hover over a specified x,y,z coordinate.

* pix-logdata.py -> Script that logs data from a computer vision thread and from a flying multicopter with a Pixhawk. DroneApi related.

* pix-takeoff.py -> Script that makes a pixhawk take off in the most secure way. DroneApi related.

[![Example take-off](http://img.youtube.com/vi/KnjYYBKLK0s/0.jpg)](http://www.youtube.com/watch?v=KnjYYBKLK0s)

* pix-goto.py -> Script that commands the vehicle to follow waypoints. 

![Example goto](http://www.aldux.net/images/goto.png "SITL of this script working")

* pix-velocity-vector.py -> Script that send the vehicle a velocity vector to form a square and diamond shape.

![Example velocity vector](http://www.aldux.net/images/velocity.png "SITL of this script working")

## Supported flight controllers:

* Multiwii boards (using MSP)

* Pixhawk / PX4 / APM (using mavlink & drone API)

## Supported companion computers: 

* Raspberry Pi (currently in use)

* oDroid U3 (taking a rest)

Note: Code is in python, so, any linux computer would be able tu run it.

## How to:

* Raspberry Pi (Rasbian)

<ol start="1"><li> Update your Pi... </li></ol>
```
sudo apt-get update
sudo apt-get upgrade
```

<ol start="2"><li> Next this one: </li></ol>
```
sudo apt-get install python-dev
```

<ol start="3"><li> More utilities: </li></ol>
```
sudo apt-get install screen python-wxgtk2.8 python-matplotlib python-opencv python-pip python-numpy python-serial
```

<ol start="4"><li> Then mavlink and mavproxy:</li></ol>
```
sudo pip install pymavlink
sudo pip install mavproxy
sudo pip install droneapi
```

<ol start="5"><li> Clone this repository:</li></ol>
```
git clone https://github.com/alduxvm/DronePilot.git
```

#### Simulation

If you're doing tests with simulation, don't forget to turn off the parameter checking inside the mavproxy console, so that you can arm the vehicle:

```
param set ARMING_CHECK 0

```

All set!! go do tests!!

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

### ssh:

/etc/ssh/ssh_config
```
ServerAliveInterval 5
ClientAliveInterval
```

### ad-hoc:

/etc/network/interfaces
```
auto lo
iface lo inet loopback
iface eth0 inet dhcp
 
auto wlan0
iface wlan0 inet static
  address 192.168.1.1
  netmask 255.255.255.0
  wireless-channel 1
  wireless-essid RPiAdHocNetwork
  wireless-mode ad-hoc
```

```
sudo ifdown wlan0
sudo ifup wlan0
```

```
sudo apt-get install dhcp3-server
```

```
ddns-update-style interim;
default-lease-time 600;
max-lease-time 7200;
authoritative;
log-facility local7;
subnet 192.168.1.0 netmask 255.255.255.0 {
  range 192.168.1.5 192.168.1.150;
}
```

### VIDEO

```
sudo modprobe -v bcm2835-v4l2
git clone https://github.com/mpromonet/h264_v4l2_rtspserver.git
cd h264_v4l2_rtspserver
cmake .
make install
h264_v4l2_rtspserver -H <height> -W <width> -F <fps>
```
