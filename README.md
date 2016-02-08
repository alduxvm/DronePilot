![Altax](https://altax.net/images/altax.png "Altax")

# Drone Pilot

Autonomous Pilot software that runs on companion computers. 

Main functionalities:

* Communication with flight controllers, including <Pixhawk, PX4, APM and MultiWii>.
* Bridge between a ground station computer and flight controller, this allows to control vehicles from a computer.
* Black box of flight data (save all the data from the flight controller and ground station).
* Position control algorithms. Uses data from a motion capture system and compute the pilot commands to keep the vehicle in a desired position.
* Uses DroneKit (when using Pixhawk and PX4) to perform advanced missions.

The objective of this repository is to do research using multirotor vehicles and learn how to develop with them.

Vehicle flying using a hover controller (mw-hover-controller.py):

![Flying quadcopter](https://altax.net/images/quad.jpg "Flying quadcopter")

This library uses a companion computer alongside a flight controller, you can check this post <https://altax.net/blog/flight-stack/>.

---

## Current scripts:

### MultiWii scripts:

* mw-joystick.py -> Send joystick commands via UDP from a ground-station running Matlab to a flight controller running MultiWii software.

[![Multiwii joystick (naze32)](http://img.youtube.com/vi/XyyfGp-IomE/0.jpg)](http://www.youtube.com/watch?v=XyyfGp-IomE)

* mw-hover-controller.py -> Computer pilot commands (using a *PID structure*) to make a Multiwii multicopter hold a specified x,y,z coordinate.


### Pixhawk scripts:

* pix-showdata.py -> Reads data from a flight controller (either SITL or real) and displays it. Dronekit related. Script that will be used first to ensure the communication with the flight controller is working.

* pix-takeoff.py -> Script that makes a pixhawk take off in a secure way. Dronekit related.

[![Example take-off](http://img.youtube.com/vi/KnjYYBKLK0s/0.jpg)](http://www.youtube.com/watch?v=KnjYYBKLK0s)

* pix-joystick.py -> Send joystick commands via UDP from a ground-station running Matlab to a pixhawk. Dronekit related.

[![Pixhawk joystick](http://img.youtube.com/vi/TkYeQ6orN8Y/0.jpg)](http://www.youtube.com/watch?v=TkYeQ6orN8Y)

* pix-logdata.py -> Script that logs data from a vehicle and a MoCap system. Dronekit related.

![Example logdata](https://altax.net/images/log.jpg "Example of a log of this script working")

* pix-goto.py -> Script that commands the vehicle to follow waypoints. 

![Example goto](https://altax.net/images/goto.jpg "SITL of this script working")

* pix-velocity-vector.py -> Script that send the vehicle a velocity vector to form a square and diamond shape.

![Example velocity vector](https://altax.net/images/velocity.jpg "SITL of this script working")

---

## Supported flight controllers:

* Multiwii boards (using MSP). Currently using on this examples a naze32.

* Pixhawk / PX4 / APM (using mavlink & DroneKit 2.0)


## Supported companion computers: 

* Raspberry Pi (B+, 2 or Zero)

* oDroid U3 

Note: Code is in python, so, any linux companion computer should be able tu run it.

---

## How to:

### Raspbian

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
sudo pip install dronekit
```

<ol start="5"><li> Clone this repository:</li></ol>
```
git clone https://github.com/alduxvm/DronePilot.git
```

Have a read here <https://altax.net/blog/how-to-dronekit-sitl/> for a tutorial on DroneKit and SITL.

--

### Ubuntu LTS

#### Install Ubuntu

* Download the Ubuntu LTS image for raspberry pi here: <https://wiki.ubuntu.com/ARM/RaspberryPi>
* Use Apple Pi-baker to create and put the image on the sd card (mac users)
* Put sd on RPI and turn it on
* Once booted, follow instructions to <https://wiki.ubuntu.com/ARM/RaspberryPi> expand file system
* Change password (optional) ```sudo passwd ubuntu```

#### Connect to internet or a network

Have in mind that this particular configuration is for the wireless setup in my laboratory, you will have to change this part so that it suits your own configuration.

* Mount a USB to install two packages: libw30, wireless-tools (packages must be for armhf)
* List devices: ```sudo fdisk -l```
* Create USB folder ```sudo mkdir /media/usb```
* Mount device ```sudo mount /dev/sda2 /media/usb```
* Navigate to the folder and install:
```
sudo dpkg -i libiw30_30-pre9-8ubuntu1_armhf.deb
sudo dpkg -1 wireless-tools_30-pre9-5ubuntu2_armhf.deb
```
* Add this information to interfaces:
```
auto wlan0
iface wlan0 inet dhcp
	wireless-essid	jws-uav-lab
	wireless-mode	Managed
	wireless-key	s:#########
```
* Reboot and you will be connected!

#### Utilities and extra software

* ```sudo apt-get install openssh-server```
* ```sudo apt-get install python-dev```
* ```sudo apt-get install screen python-wxgtk2.8 python-matplotlib python-opencv python-pip python-numpy python-serial python-twisted htop git```
* ```sudo pip install pymavlink```
* ```sudo pip install mavproxy```
* ```sudo pip install dronekit```

#### For higher UART speed

Edit /boot/config.txt file, and add to the end:
```
# Higher UART Speed
init_uart_baud=921600
init_uart_clock=14745600
```

---

## Caution

This code is still under heavy development, everyday I add and remove stuff. Proceed with caution.

---

## Social networks:

You can follow us in this URL's:

* [Altax.net](http://altax.net/)
* [Altax.net Facebook](https://www.facebook.com/AltaxConsulting/)

---

## Utilities:

#### updateDronePilot shell script 

```
#!/bin/bash

echo "Deleting previous directory..."
rm -rf DronePilot/
echo "Done."
echo "Cloning DronePilot repository: "
git clone https://github.com/alduxvm/DronePilot.git
#echo "Copying mavinit.scr to proper directory..."
#mkdir DronePilot/TestQuad
#mv DronePilot/mavinit.scr DronePilot/TestQuad/
#echo "Done."
echo "Ready to do: mavproxy.py --master=/dev/ttyAMA0 --baudrate 115200 --aircraft TestQuad"
echo "Dont forget to go inside the DronePilot directory."
```

#### ssh:

/etc/ssh/ssh_config
```
ServerAliveInterval 5
ClientAliveInterval 5
```

#### ad-hoc:

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
/etc/dhcp/dhcpd.conf
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

In case of failure starting DHCP:
```
sudo service ifplugd stop
```

#### Video

```
sudo modprobe -v bcm2835-v4l2
git clone https://github.com/mpromonet/h264_v4l2_rtspserver.git
cd h264_v4l2_rtspserver
cmake .
make install
h264_v4l2_rtspserver -H <height> -W <width> -F <fps>
```

To start:
```
rtsp://192.168.1.1:8554/unicast
```

#### Low latency video

* Same DHCP configuration as before

* Install "hostapd" 
```
sudo apt-get install hostapd netcat-traditional

```

* Interfaces:
```
auto wlan0
iface wlan0 inet static
  address 85.85.85.1
  netmask 255.255.255.0
```

* /etc/hostapd/hostapd.conf 
```
interface=wlan0
ssid=HDFPV
hw_mode=g
channel=6
auth_algs=1
wmm_enabled=0
```

* /etc/default/hostapd
```
DAEMON_CONF="/etc/hostapd/hostapd.conf"
```

* On rpi, create fifo file:
```
mkfifo fifo.500 
```

* On client (mac):
```
brew install netcat
brew install mplayer
```

* To start (mac):
```
netcat -l -p 5000 | mplayer -fps 60 -cache 1024 -
```

* To start (rpi):
```
cat fifo.500 | nc.traditional 85.85.85.6 5000 &
/opt/vc/bin/raspivid -o fifo.500 -t 0 -b 50000
```


