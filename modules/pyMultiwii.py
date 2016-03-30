#!/usr/bin/env python

"""multiwii.py: Handles Multiwii Serial Protocol."""

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2014 Aldux.net"

__license__ = "GPL"
__version__ = "1.5"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"


import serial, time, struct


class MultiWii:

    """Multiwii Serial Protocol message ID"""
    """ notice: just attitude, rc channels and raw imu, set raw rc are implemented at the moment """
    IDENT = 100
    STATUS = 101
    RAW_IMU = 102
    SERVO = 103
    MOTOR = 104
    RC = 105
    RAW_GPS = 106
    COMP_GPS = 107
    ATTITUDE = 108
    ALTITUDE = 109
    ANALOG = 110
    RC_TUNING = 111
    PID = 112
    BOX = 113
    MISC = 114
    MOTOR_PINS = 115
    BOXNAMES = 116
    PIDNAMES = 117
    WP = 118
    BOXIDS = 119
    RC_RAW_IMU = 121
    SET_RAW_RC = 200
    SET_RAW_GPS = 201
    SET_PID = 202
    SET_BOX = 203
    SET_RC_TUNING = 204
    ACC_CALIBRATION = 205
    MAG_CALIBRATION = 206
    SET_MISC = 207
    RESET_CONF = 208
    SET_WP = 209
    SWITCH_RC_SERIAL = 210
    IS_SERIAL = 211
    DEBUG = 254


    """Class initialization"""
    def __init__(self, serPort):

        """Global variables of data"""
        self.rcChannels = {'roll':0,'pitch':0,'yaw':0,'throttle':0,'elapsed':0,'timestamp':0}
        self.rawIMU = {'ax':0,'ay':0,'az':0,'gx':0,'gy':0,'gz':0,'elapsed':0,'timestamp':0}
        self.motor = {'m1':0,'m2':0,'m3':0,'m4':0,'elapsed':0,'timestamp':0}
        self.attitude = {'angx':0,'angy':0,'heading':0,'elapsed':0,'timestamp':0}
        self.message = {'angx':0,'angy':0,'heading':0,'roll':0,'pitch':0,'yaw':0,'throttle':0,'elapsed':0,'timestamp':0}
        self.temp = ();
        self.temp2 = ();
        self.elapsed = 0
        self.PRINT = 1

        self.ser = serial.Serial()
        self.ser.port = serPort
        self.ser.baudrate = 115200
        self.ser.bytesize = serial.EIGHTBITS
        self.ser.parity = serial.PARITY_NONE
        self.ser.stopbits = serial.STOPBITS_ONE
        self.ser.timeout = 0
        self.ser.xonxoff = False
        self.ser.rtscts = False
        self.ser.dsrdtr = False
        self.ser.writeTimeout = 2
        """Time to wait until the board becomes operational"""
        wakeup = 2
        try:
            self.ser.open()
            if self.PRINT:
                print "Waking up board on "+self.ser.port+"..."
            for i in range(1,wakeup):
                if self.PRINT:
                    print wakeup-i
                    time.sleep(1)
                else:
                    time.sleep(1)
        except Exception, error:
            print "\n\nError opening "+self.ser.port+" port.\n"+str(error)+"\n\n"

    """Function for sending a command to the board"""
    def sendCMD(self, data_length, code, data):
        checksum = 0
        total_data = ['$', 'M', '<', data_length, code] + data
        for i in struct.pack('<2B%dh' % len(data), *total_data[3:len(total_data)]):
            checksum = checksum ^ ord(i)
        total_data.append(checksum)
        try:
            b = None
            b = self.ser.write(struct.pack('<3c2B%dhB' % len(data), *total_data))
        except Exception, error:
            #print "\n\nError in sendCMD."
            #print "("+str(error)+")\n\n"
            pass

    """Function for sending a command to the board and receive attitude"""
    """
    Modification required on Multiwii firmware to Protocol.cpp in evaluateCommand:

    case MSP_SET_RAW_RC:
      s_struct_w((uint8_t*)&rcSerial,16);
      rcSerialCount = 50; // 1s transition 
      s_struct((uint8_t*)&att,6);
      break;

    """
    def sendCMDreceiveATT(self, data_length, code, data):
        checksum = 0
        total_data = ['$', 'M', '<', data_length, code] + data
        for i in struct.pack('<2B%dh' % len(data), *total_data[3:len(total_data)]):
            checksum = checksum ^ ord(i)
        total_data.append(checksum)
        try:
            start = time.time()
            b = None
            b = self.ser.write(struct.pack('<3c2B%dhB' % len(data), *total_data))
            while True:
                header = self.ser.read()
                if header == '$':
                    header = header+self.ser.read(2)
                    break
            datalength = struct.unpack('<b', self.ser.read())[0]
            code = struct.unpack('<b', self.ser.read())
            data = self.ser.read(datalength)
            temp = struct.unpack('<'+'h'*(datalength/2),data)
            self.ser.flushInput()
            self.ser.flushOutput()
            elapsed = time.time() - start
            self.attitude['angx']=float(temp[0]/10.0)
            self.attitude['angy']=float(temp[1]/10.0)
            self.attitude['heading']=float(temp[2])
            self.attitude['elapsed']=round(elapsed,3)
            self.attitude['timestamp']="%0.2f" % (time.time(),) 
            return self.attitude
        except Exception, error:
            #print "\n\nError in sendCMDreceiveATT."
            #print "("+str(error)+")\n\n"
            pass

    """Function to arm / disarm """
    """
    Modification required on Multiwii firmware to Protocol.cpp in evaluateCommand:

    case MSP_SET_RAW_RC:
      s_struct_w((uint8_t*)&rcSerial,16);
      rcSerialCount = 50; // 1s transition 
      s_struct((uint8_t*)&att,6);
      break;

    """
    def arm(self):
        timer = 0
        start = time.time()
        while timer < 0.5:
            data = [1500,1500,2000,1000]
            self.sendCMD(8,MultiWii.SET_RAW_RC,data)
            time.sleep(0.05)
            timer = timer + (time.time() - start)
            start =  time.time()

    def disarm(self):
        timer = 0
        start = time.time()
        while timer < 0.5:
            data = [1500,1500,1000,1000]
            self.sendCMD(8,MultiWii.SET_RAW_RC,data)
            time.sleep(0.05)
            timer = timer + (time.time() - start)
            start =  time.time()

    """Function to receive a data packet from the board"""
    def getData(self, cmd):
        try:
            start = time.time()
            self.sendCMD(0,cmd,[])
            while True:
                header = self.ser.read()
                if header == '$':
                    header = header+self.ser.read(2)
                    break
            datalength = struct.unpack('<b', self.ser.read())[0]
            code = struct.unpack('<b', self.ser.read())
            data = self.ser.read(datalength)
            temp = struct.unpack('<'+'h'*(datalength/2),data)
            self.ser.flushInput()
            self.ser.flushOutput()
            elapsed = time.time() - start
            if cmd == MultiWii.ATTITUDE:
                self.attitude['angx']=float(temp[0]/10.0)
                self.attitude['angy']=float(temp[1]/10.0)
                self.attitude['heading']=float(temp[2])
                self.attitude['elapsed']=round(elapsed,3)
                self.attitude['timestamp']="%0.2f" % (time.time(),) 
                return self.attitude
            elif cmd == MultiWii.RC:
                self.rcChannels['roll']=temp[0]
                self.rcChannels['pitch']=temp[1]
                self.rcChannels['yaw']=temp[2]
                self.rcChannels['throttle']=temp[3]
                self.rcChannels['elapsed']=round(elapsed,3)
                self.rcChannels['timestamp']="%0.2f" % (time.time(),)
                return self.rcChannels
            elif cmd == MultiWii.RAW_IMU:
                self.rawIMU['ax']=float(temp[0])
                self.rawIMU['ay']=float(temp[1])
                self.rawIMU['az']=float(temp[2])
                self.rawIMU['gx']=float(temp[3])
                self.rawIMU['gy']=float(temp[4])
                self.rawIMU['gz']=float(temp[5])
                self.rawIMU['elapsed']=round(elapsed,3)
                self.rawIMU['timestamp']="%0.2f" % (time.time(),)
                return self.rawIMU
            elif cmd == MultiWii.MOTOR:
                self.motor['m1']=float(temp[0])
                self.motor['m2']=float(temp[1])
                self.motor['m3']=float(temp[2])
                self.motor['m4']=float(temp[3])
                self.motor['elapsed']="%0.3f" % (elapsed,)
                self.motor['timestamp']="%0.2f" % (time.time(),)
                return self.motor
            else:
                return "No return error!"
        except Exception, error:
            #print error
            pass

    """Function to receive a data packet from the board. Note: easier to use on threads"""
    def getDataInf(self, cmd):
        while True:
            try:
                start = time.clock()
                self.sendCMD(0,cmd,[])
                while True:
                    header = self.ser.read()
                    if header == '$':
                        header = header+self.ser.read(2)
                        break
                datalength = struct.unpack('<b', self.ser.read())[0]
                code = struct.unpack('<b', self.ser.read())
                data = self.ser.read(datalength)
                temp = struct.unpack('<'+'h'*(datalength/2),data)
                elapsed = time.clock() - start
                self.ser.flushInput()
                self.ser.flushOutput()
                if cmd == MultiWii.ATTITUDE:
                    self.attitude['angx']=float(temp[0]/10.0)
                    self.attitude['angy']=float(temp[1]/10.0)
                    self.attitude['heading']=float(temp[2])
                    self.attitude['elapsed']="%0.3f" % (elapsed,)
                    self.attitude['timestamp']="%0.2f" % (time.time(),)
                elif cmd == MultiWii.RC:
                    self.rcChannels['roll']=temp[0]
                    self.rcChannels['pitch']=temp[1]
                    self.rcChannels['yaw']=temp[2]
                    self.rcChannels['throttle']=temp[3]
                    self.rcChannels['elapsed']="%0.3f" % (elapsed,)
                    self.rcChannels['timestamp']="%0.2f" % (time.time(),)
                elif cmd == MultiWii.RAW_IMU:
                    self.rawIMU['ax']=float(temp[0])
                    self.rawIMU['ay']=float(temp[1])
                    self.rawIMU['az']=float(temp[2])
                    self.rawIMU['gx']=float(temp[3])
                    self.rawIMU['gy']=float(temp[4])
                    self.rawIMU['gz']=float(temp[5])
                    self.rawIMU['elapsed']="%0.3f" % (elapsed,)
                    self.rawIMU['timestamp']="%0.2f" % (time.time(),)
                elif cmd == MultiWii.MOTOR:
                    self.motor['m1']=float(temp[0])
                    self.motor['m2']=float(temp[1])
                    self.motor['m3']=float(temp[2])
                    self.motor['m4']=float(temp[3])
                    self.motor['elapsed']="%0.3f" % (elapsed,)
                    self.motor['timestamp']="%0.2f" % (time.time(),)
            except Exception, error:
                pass

    """Function to ask for 2 fixed cmds, attitude and rc channels, and receive them. Note: is a bit slower than others"""
    def getData2cmd(self, cmd):
        try:
            start = time.time()
            self.sendCMD(0,self.ATTITUDE,[])
            while True:
                header = self.ser.read()
                if header == '$':
                    header = header+self.ser.read(2)
                    break
            datalength = struct.unpack('<b', self.ser.read())[0]
            code = struct.unpack('<b', self.ser.read())
            data = self.ser.read(datalength)
            temp = struct.unpack('<'+'h'*(datalength/2),data)
            self.ser.flushInput()
            self.ser.flushOutput()

            self.sendCMD(0,self.RC,[])
            while True:
                header = self.ser.read()
                if header == '$':
                    header = header+self.ser.read(2)
                    break
            datalength = struct.unpack('<b', self.ser.read())[0]
            code = struct.unpack('<b', self.ser.read())
            data = self.ser.read(datalength)
            temp2 = struct.unpack('<'+'h'*(datalength/2),data)
            elapsed = time.time() - start
            self.ser.flushInput()
            self.ser.flushOutput()

            if cmd == MultiWii.ATTITUDE:
                self.message['angx']=float(temp[0]/10.0)
                self.message['angy']=float(temp[1]/10.0)
                self.message['heading']=float(temp[2])
                self.message['roll']=temp2[0]
                self.message['pitch']=temp2[1]
                self.message['yaw']=temp2[2]
                self.message['throttle']=temp2[3]
                self.message['elapsed']=round(elapsed,3)
                self.message['timestamp']="%0.2f" % (time.time(),) 
                return self.message
            else:
                return "No return error!"
        except Exception, error:
            print error
