"""
This code is design to run in RPi connected (via USB) with Arduino

Creator:            Mana Saedan
Date Last Edited:   27 January 2017

"""
import time
import sys
import serial
import thread
import serial.tools.list_ports

"""
   Aircraft RC channel mapping
   Channel 1 = Roll
   Channel 2 = Pitch
   Channel 3 = Throttle
   Channel 4 = Yaw
   Channel 5 = Flight Mode
"""
class HardOverride:
    def __init__(self):
        self.Ch1 = 1500
        self.Ch2 = 1500
        self.Ch3 = 1000
        self.Ch4 = 1500
        self.Ch5 = 1000
    
    def Connect(self):
        self.Port = None
        ListPortInfo = serial.tools.list_ports.comports()
        for list in ListPortInfo:
            if (list.product != None):
                if (list.product.find("duino") != -1):
                    try:
                        #Try to connect to any port with "Arduino" name
                        ser = serial.Serial(list.device, timeout=2.0, baudrate=115200)
                            
                        #Check the reply. The arduino "hardoverride" does not return anythings
                        if (len(ser.readline()) ==0):
                            self.Port = ser
                            break
                    
                    except ser.SerialTimeoutException:
                        #If the serial does not return message. Assume it is a "hardoverride"
                        self.Port = ser
                        break
        
        #Display message if no hardoverride found
        if (self.Port == None):
            print "ERROR: No hard override device found!\r"
            return False
        else:
            print "Connect to Hardoverride at %s\r" %list.device
        return True
            

    def Send(self, Ch1, Ch2, Ch3, Ch4, Ch5):
        if (self.Port == None):
            return
        
        self.Ch1 = Ch1
        self.Ch2 = Ch2
        self.Ch3 = Ch3
        self.Ch4 = Ch4
        self.Ch5 = Ch5
    
        """
        Byte 0: 0x20    ---> Header byte mark
        Byte 1: 0x40    ---> Header byte mark
        Byte 2: TimeStamp   ---> Package timestamp
        Byte 3: Override    ---> RC channel override command
        Byte 4: Ch1-Lo      ---> Lower byte of RC channel 1
        Byte 5: Ch1-Hi      ---> Higher byte of RC channel 1
        Byte 6: Ch2-Lo      ---> Lower byte of RC channel 2
        Byte 7: Ch2-Hi      ---> Higher byte of RC channel 2
        Byte 8: Ch3-Lo      ---> Lower byte of RC channel 3
        Byte 8: Ch3-Hi      ---> Higher byte of RC channel 3
        Byte10: Ch4-Lo      ---> Lower byte of RC channel 4
        Byte11: Ch4-Hi      ---> Higher byte of RC channel 4
        Byte12: Ch5-Lo      ---> Lower byte of RC channel 5
        Byte13: Ch5-Hi      ---> Higher byte of RC channel 5
        Byte14: ChkSum-Lo   ---> Lower byte of check sum
        Byte15: ChkSum-Hi   ---> Higher byte of check sum
        """
        
        rcBytes = [0 for i in range(16)]
        rcBytes[0] = 0x20
        rcBytes[1] = 0x40
        rcBytes[2] = 1
        rcBytes[3] = 0xFF
        rcBytes[4] = int(Ch1 & 0xFF)
        rcBytes[5] = int((Ch1>>8) & 0xFF)
        rcBytes[6] = int(Ch2 & 0xFF)
        rcBytes[7] = int((Ch2>>8) & 0xFF)
        rcBytes[8] = int(Ch3 & 0xFF)
        rcBytes[9] = int((Ch3>>8) & 0xFF)
        rcBytes[10] = int(Ch4 & 0xFF)
        rcBytes[11] = int((Ch4>>8) & 0xFF)
        rcBytes[12] = int(Ch5 & 0xFF)
        rcBytes[13] = int((Ch5>>8) & 0xFF)
    
        #Calculate check sum
        chksum = 0xFFFF
        for i in range(0,14):
            chksum -= rcBytes[i]
        
        rcBytes[14] = int(chksum & 0xFF)
        rcBytes[15] = int((chksum>>8) & 0xFF)

        #Send serial data
        for i in range(0,16):
            self.Port.write(str(chr(rcBytes[i])))
            
    def Close(self):
        self.Port.close()

"""
---------------- End of Class HardOverride --------------
"""
