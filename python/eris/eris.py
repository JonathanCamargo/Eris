""" Utilities for controlling eris from python"""
from serial import Serial

import cobs.cobs as cobs
import struct

import math

from time import sleep

import struct

from construct import *
import binascii
import sys


class Eris:
        """ Eris (python) is the driver for the Eris firmware on the microcontroller"""
        port=None

        features=None
        ispython2=True
        if (sys.version_info > (3, 0)):
            ispython2=False

        def __init__(self,features,format,port='/dev/ttyACM0'):
            """Create an eris object features,types,lengths,port)\n
            pass a list of features (type of data that you want to activate in the streaming
            format is a c format to interpreting the streaming packets.
            Optional port is the serial port to be used (defaults to ttyACM0)\n
            e.g. Eris(['RMS'],format,'/dev/ttyACM0')"""

            self.features=features
            self.buffer=b'' #Buffer to store incomplete packet data in the serial-RX

            self.port=Serial(port,baudrate=12000000,timeout=1.0)
            self.format=Sequence(*format)
            #Open port
            sleep(1)
            #Inform uC what type of data we want to retrieve
            self.sendCommand('S_OFF') #Stop any previous streaming
            #Get what version of eris firmware
            self.sendCommand('INFO')
            for i in range(3):
                if self.port.in_waiting<1:
                    sleep(0.1)
                else:
                    response=self._readString()
                    print(response) #TODO parse info on firmware and enable disable functionality?
                    break

            cmd='S_F '+' '.join(features)
            self.sendCommand(cmd)
            sleep(0.5)
            print(self._readString())
            sleep(0.5)
            print(self._readString())
            print('Eris is ready')

        def start(self):
            '''
            start()
            Start streaming with Eris
            '''
            self.sendCommand('S_ON')

        def stop(self):
            '''
            stop()
            Stop streaming with Eris
            '''
            self.sendCommand('S_OFF')
            sleep(0.5)
            self.port.close()

        def sendCommand(self,command):
            cmd=''.join([command,'\n'])
            self.port.write(str.encode(cmd))
            self.port.flush()

        def read(self):
            '''Read the data in the RX buffer based on the struct definitions
            Returns a list of samples where each sample holds data as defined in the setup in
            unpacked form.
            '''
            data={'D':[],'R':[],'C':[],'T':[]}
            if self.port.in_waiting:
                a=self.port.read(self.port.in_waiting)
                self.buffer=self.buffer+a
                packets=self.buffer.split(b'\0') #Split by packet end char
                #if (len(packets)<2):
                #    return data

                self.buffer=packets[-1]
                packets=packets[0:-1]
                #Read each packet using cobs decoded
                for packet in packets:
                    try:
                        decoded=cobs.decode(packet)
                    except Exception as ex:
                        print('Problem interpreting packet:')
                        print(ex)
                        print(packet)
                        continue
                    #First byte is type of packet:
                    # 'D' is for data
                    # 'T' is for text
                    if self.ispython2:
                        a=decoded[0]
                    else:
                        a=chr(decoded[0])
                    if a=='D':
                        data['D'].append(decoded[1:])
                    elif a=='R':
                        data['R'].append(decoded[1:])
                    elif a=='C':
                        data['C'].append(decoded[1:])
                    elif a=='T':
                        print(decoded[1:])
                        data['T'].append(decoded[1:].decode('ascii'))
                    else:
                        print('Unsupported Packet type:'),
                        print(decoded)
            return data

        def parse(self,unpacked):
            '''
            Transformed a list of unpacked data in a more readable form.
            '''
            content=self.format.parse(unpacked)
            out={}
            for i,feature in enumerate(self.features):
                out[feature]=content[i]
            return out

        def _readString(self):
            #Read whatever is in in serial as string and clear the buffer
            a=self.port.read(self.port.in_waiting)
            self.buffer=self.buffer+a
            packets=self.buffer.split(b'\0')
            self.buffer=b''
            return packets[-1]
