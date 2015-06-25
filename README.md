# Drone Pilot

 Pilot software (running on companion computers) for the flight controller autopilot's. It can control and fly several flight controllers, including Pixhawk's, APM's and MultiWii's.

## Current scripts:

* mw-joystick-test.py -> Send joystick commands via UDP from a ground-station running Matlab to a FC running MultiWii software.

* mw-hover-controller.py -> Calculate commands to make a Multiwii multicopter hover over a specified x,y,z coordinate.

* pix-logdata.py -> Script that logs data from a computer vision thread and from a flying multicopter with a Pixhawk. DroneApi related.

* pix-takeoff.py -> Script that makes a pixhawk take off in the most secure way. DroneApi related.

* pix-goto.py -> Script that commands the vehicle to follow waypoints. 

* pix-velocity-vector.py -> Script that send the vehicle a velocity vector to form a square and diamond shape.

## Supported flight controllers:

* Multiwii boards (using MSP)

* Pixhawk / PX4 / APM (using mavlink & drone API)

## Supported companion computers: 

* Raspberry Pi (currently in use)

* oDroid U3 (taking a rest)

Note: Code is in python, so, any linux computer would be able tu run it.

## How to:

* Raspberry Pi (Rasbian)

<ol>
<li> Update your Pi </li>
```
sudo apt-get update
sudo apt-get upgrade
```

<li> Next this one: </li>
```
sudo apt-get install python-dev
```

<li> More utilities: </li>
```
sudo apt-get install screen python-wxgtk2.8 python-matplotlib python-opencv python-pip python-numpy python-serial
```

<li> Then mavlink and mavproxy: </li>
```
sudo pip install pymavlink
sudo pip install mavproxy
sudo pip install droneapi
```

<li> Clone this repository: </li>
```
git clone https://github.com/alduxvm/DronePilot.git
```
</ol>

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

### FPV

```
sudo apt-get install mplayer netcat
cd /opt/vc/src/hello_pi
make –C libs/ilclient
make –C libs/vgfont
cd /opt/vc/src/hello_pi/hello_video
make
cd ~
```
