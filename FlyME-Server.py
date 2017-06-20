"""
The code is designed to be a server running on RPi waiting for client to initiate drone launching
Creator:            Mana Saedan
Date Last Edited:   4 Febuary 2017

Development Logs:
    - Server waiting for client to connect
    - Data format from client (32-byte-string):  mode,height,roll,pitch,yaw,angle,cheksum,...
                                                 Mode:             1 byte
                                                 Height:           5 bytes in unit of millimeter
                                                 Roll, Pitch, Yaw: Each has 3 bytes number from 0-200
                                                 Angle:            3 bytes indicate angle if smarphone (front) with respect to north pole
                                                 Checksum:
                                                 
    - Mode byte meaning
      Bit 0 = Arm/Disarm
      Bit 1 = Takeoff/Landing
      Bit 2,3 ==> 00=Hoover, 01=Self frame, 10=Earth fame, 11=Smartphone frame
"""

import sys
import socket
from QuadCopter import QuadCopter

#Intialize FlyME class
aircraft = QuadCopter()

def DecodeTCP(Mode, Height, Roll, Pitch, Yaw, Angle):
    global aircraft
    #Check whether the aircraft is armed
    if (Mode & 0x01):
        if not(aircraft.isArm()):
            print "Start"
            aircraft.Start()
        return
#End DecodeFlightMode

############################ Main Program ############################
#Connect with aircraft devices, e.g. pixhawk, hard override, lidar,...
if (aircraft.ConnectDevices(False, False) != 0):
    print "Unable connect aircraft devices"
    sys.exit(1)

#Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Bind the socket to the port
server_address = ('localhost', 10000)
print >>sys.stderr, 'Starting up on %s port %s' % server_address
sock.bind(server_address)
#Listen for incoming connections
sock.listen(1)
Status = True
while Status:
    #Wait for a connection
    print >>sys.stderr, 'Waiting for a connection'
    connection, client_address = sock.accept()
    try:
        #Display client address
        print >>sys.stderr, 'Connection from', client_address
        
        #Loop indefinite waiting for client
        while True:
            #Read 23 bytes from client
            txt_data = connection.recv(32)
            
            #Exit loop when no data receive or client is disconnected
            if not txt_data: break
            
            #Upon receiving 32 bytes, decode text data
            if len(txt_data)>0:
                
                #Remove carrirage and line feed
                txt_data = txt_data.replace('\n', '').replace('\r', '')
                
                #Split data from text
                data =  txt_data.split(',')
                
                if (len(data) != 8):
                    print >>sys.stderr, 'Data receive error'
                else:
                    #Check the package validity, data[6]
                    chksum=0
                    #Go through data from 'Mode' to 'Angle'
                    for i in range(6):
                        print data[i]
                        for each_char in data[i]:
                            chksum = chksum + int(each_char)
                
                    #When checksum is valid process the aircraft command here
                    if (chksum == int(data[6])):
                        DecodeTCP(int(data[0]), int(data[1]), int(data[2]), int(data[3]), int(data[4]), int(data[5]))

    finally:
        # Clean up the connection
        print >>sys.stderr, 'Server closed.'
        connection.close()
