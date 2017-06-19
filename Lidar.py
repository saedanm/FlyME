import sys
import math
import serial
import serial.tools.list_ports


class Lidar():
    def __init__(self):
        deg2rad = math.pi/180.0
        self.Port = None
        self.LidarAngle = 1.71428571428571
        #Offset angle from sensor X-axis
        self.LidarAngleOffset = -5.14285714285713
        self.LidarSin = []
        self.LidarCos = []
        self.LidarRange = []
        self.LidarX = []
        self.LidarY = []
        for i in range(210):
            self.LidarRange.append(0)
            self.LidarX.append(0)
            self.LidarY.append(0)
            #This to get the angle from range index
            myangle = (self.LidarAngleOffset +i*self.LidarAngle)*deg2rad
            self.LidarSin.append(math.sin(myangle))
            self.LidarCos.append(math.cos(myangle))

    def Connect(self):
        if (self.Port != None):
            self.Port.close()
        
        self.Port = None
        ListPortInfo = serial.tools.list_ports.comports()
        for list in ListPortInfo:
            if (list.product != None):
                if (list.product.find("duino") != -1):
                    try:
                        #Try to connect to any port with "Arduino" name
                        ser = serial.Serial(list.device, timeout=2.0, baudrate=115200)
                        #Check the reply.
                        #print ser.readline()
                        if (len(ser.readline()) !=0):
                            self.Port = ser
                            break
                    
                    except ser.SerialTimeoutException:
                        pass

        #Display message if no hardoverride found
        if (self.Port != None):
            print "Connect Lidar at %s" %list.device
            return True
        else:
            print "No lidar device found!\r"
            return False

    def Read(self, ResX, ResY): #ResX and ResY units are in centimeter
        if (self.Port == None):
            return 0
        
        #Read one line data from serial port
        text_data =  self.Port.readline()
        #Remove carrirage and line feed
        text_data = text_data.replace('\n', '').replace('\r', '')
        #Split angle and range data from text
        
        data =  text_data.split(',')
        index = -1
        range_read = False
        if (len(data) >=2):
            try:
                index  = int(data[0])
                if (index>=0 and index <210):
                    try:
                        self.LidarRange[index] = float(data[1])
                        range_read = True
                    except ValueError:
                        self.LidarRange[index] = 0
            except ValueError:
                pass
    
        #If range data is not valid, do not convert the range value
        if (range_read == False):
            return -2

        #Convert range to X and Y coordinate
        lx = self.LidarRange[index]*self.LidarCos[index]
        ly = self.LidarRange[index]*self.LidarSin[index]
        
        #Digitize Range into X and Y coordinate if ResX or RexY > 0.0
        if (ResX>0.0):
            self.LidarX[index] = int(lx/ResX +0.5)
        else:
            self.LidarX[index] = int(lx)

        if (ResY>0.0):
            self.LidarY[index] = int(ly/ResY +0.5)
        else:
            self.LidarY[index] = int(ly)

        return index
            
    def Reset(self):
        #Reset Lidar Range value to zero
        for i in range(210):
            self.LidarRange[i] = 0.0
            self.LidarX[i]     = 0
            self.LidarY[i]     = 0

    def Close(self):
        self.Port.close()
"""
----------------------- End of Lidar class ---------------------------
"""
