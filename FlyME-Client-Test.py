"""
    The code is designed to be a client connect to server running in RPi. 
    
    Creator:            Mana Saedan
    Date Last Edited:   28 Febuary 2017
    
    Development Logs:

"""

import socket
import sys

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect the socket to the port on the server given by the caller
server_address = (sys.argv[1], 10000)
print >>sys.stderr, 'Connecting to %s port %s' % server_address
sock.connect(server_address)

try:
    message = '001,00000,000,000,000,000,001,00'
    print >>sys.stderr, 'Sending "%s"' % message
    sock.sendall(message)


finally:
    sock.close()
