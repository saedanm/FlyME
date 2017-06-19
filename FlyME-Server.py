"""
The code is designed to be a server running on RPi waiting for client to initiate drone launching
Creator:            Mana Saedan
Date Last Edited:   4 Febuary 2017

Development Logs:
    - Server waiting for client to connect
"""

import sys
import socket
from QuadCopter import QuadCopter

#Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the port
server_address = ('localhost', 10000)

print >>sys.stderr, 'Starting up on %s port %s' % server_address

sock.bind(server_address)

#Listen for incoming connections
sock.listen(1)

Status = True

#Intialize FlyME class
aircraft = QuadCopter()

oldCommand = 0
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
                
                print txt_data
                
                #Split data from text
                #As of 19 June 2017:
                #txt_data = 'Mode,Height,Roll,Pitch,Yaw,Angle,CheckSum,...
                data =  txt_data.split(',')
                
                if (len(data) != 8):
                    print >>sys.stderr, 'Data receive error'
                else:
                    #Check the package validity, data[6]
                    chksum=0
                    #Go through data from 'Mode' to 'Angle'
                    for i in range(6):
                        for each_char in data[i]:
                            chksum = chksum + int(each_char)
                
                    #When checksum is valid process the aircraft command here
                    if (chksum == int(data[6])):
                        print >>sys.stderr, 'Good Job!'

    finally:
        # Clean up the connection
        print >>sys.stderr, 'Server closed.'
        connection.close()
