"""
Author: S. Dong University of Leeds
Date: 11/08/2021
Version: 0.1

This is the module script for receiving data from IMU sensor via UDP transfer
The IMU data are sent to MATLAB via BLE
"""

import struct
import socket


class udpReceiver():
    def __init__(self, UDP_IP = "127.0.0.1", UDP_PORT = 5065):
        self.UDP_IP = UDP_IP
        self.UDP_PORT = UDP_PORT
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.UDP_IP, self.UDP_PORT))

    def recString(self, code='utf-8', bufsize=1024):
        data, addr = self.sock.recvfrom(bufsize)
        received = data.decode(code)
        return received

    def recFloat(self, bufsize=1024):
        data, addr = self.sock.recvfrom(bufsize)
        received = struct.unpack('f', data)
        received = list(received)
        return received