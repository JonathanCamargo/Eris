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
        """python Eris is the host-side driver for the Eris firmware on the microcontroller.
        Eris is an Arduino firmware framework for real-time data acquisition, streaming,
        and control built on an RTOS (ChibiOS or FreeRTOS).

        The firmware accepts serial commands to configure and trigger actions, and streams
        COBS-framed packets back. Three packet types are defined: 'D' (data), 'T' (text),
        and 'E' (error).

        The Data packet is a concatenation of a uint8 (len) and a user-defined record type
        described using Construct (pip install construct). The construct definition and the
        firmware's typedef MUST be consistent so the byte stream is interpreted correctly.


        How to use:
        1. Upload a flavor to the microcontroller and verify that it accepts the streaming
           configuration command S_F (Set Features):

        	In the serial monitor try: S_F <feature1> <feature2> ... <featureN>

        	<featureN> matches a feature name registered in the flavor's streaming.cpp,
        	e.g. S_F SINE FSR EMG.

           Eris responds with "Features Ready" when the configuration is accepted.

        2. In your Python code create an Eris object. Multiple objects can interface with
           multiple microcontrollers (just use different serial ports).

        	dataNames=['dataname1','dataname2',...]
        	format=[dataName1_format,dataName2_format, ...]
        	e=Eris(features,format)

        3.a In your python code keep reading eris continuously and execute actions with the output. The output of the read function contains a dictionary with the latest list of received 'D' packets, and 'T' packets.

           while True:
           	out=e.read2()
           	print(out['T']) # // prints all a list with all the text that was received
           	print(out['D']) # // prints the

        3.b (Alternative) If your computer is not that fast you might want to selectively process data from the queue in a more efficient strategy than fifo. For this, use the read_raw() function instead. This will only decode the packages to binary but won't parse the data inside. That way you save precious time and decide what to do with the binary data in your own code.


            while True:
            	out=e.read_raw()
            	for tpacket in out['T']:
            		doSomething(tpacket)


        """

        port=None

        features=None
        ispython2=True
        if (sys.version_info > (3, 0)):
            ispython2=False

        def __init__(self,features,format,port='/dev/ttyACM0'):
            """Create an Eris object (features, format, port).
            features: list of feature names to activate in the streaming, matching the
                      names accepted by S_F in the firmware's streaming.cpp.
            format:   list of construct/struct format strings used to decode each feature.
            port:     serial port. Default '/dev/ttyACM0'. On Windows use 'COM3' etc.
            e.g. Eris(['SineWave'], ['float'], '/dev/ttyACM0')
            """

            if type(features) != list:
                features=[features]

            if type(format) != list:
                format=[format]
            self.features=features
            self.port=Serial(port,baudrate=12000000,timeout=1.0)

	    # Generate the full DFormat according to the requested features and their individual
	    #format
            self._DFormat=None
            self._setDformat(features,format)

            self.buffer=b'' #Buffer to store incomplete packet data in the serial-RX
            self.packetTypes=['D','T','E']
            self.packetFunctions=[self.parseD,self.parseT,self.parseE]

            self._lastPackets=dict()
            self._clearLastPackets()


            #Open port
            sleep(0.1)
            #Inform uC what type of data we want to retrieve
            self.sendCommand('S_OFF') #Stop any previous streaming
            #Get what version of eris firmware
            self.sendCommand('INFO')
            for i in range(3):
                if self.port.in_waiting<1:
                    sleep(0.1)
                else:
                    response=self._readString()
                    print(response)
                    #TODO parse info on firmware and enable disable functionality?
                    break

            cmd='S_F '+' '.join(features)
            self.sendCommand(cmd)
            sleep(0.1)
            print(self._readString())
            sleep(0.1)
            print(self._readString())
            print('Eris initialized')

        def _setDformat(self,features,format):
            ''' The full format of a D packet consists of a struct of
            <uint8 len><datatype data> concatenated for as many features
            as established on initialization.
            '''
            dformats=[]
            for i,feature in enumerate(features):
                thisFormat=Struct(
                    "len" / Int8ub,
                    feature / Array(this.len,format[i])
                )
                dformats.append(thisFormat)
            self._Dformat=Sequence(*dformats)

        def emptyPackets(self):
            emptyPackets=dict()
            for packetType in self.packetTypes:
            	emptyPackets[packetType]=[]
            return emptyPackets

        def _clearLastPackets(self):
            self._lastPackets=self.emptyPackets()

        def parseD(self,decodedPacket):
            ''' Parse data from a D packet
            a D packet contains data with format defined in self._Dformat
            use that to parse.
            '''
            content=self._Dformat.parse(decodedPacket)
            out=dict()
            for i,feature in enumerate(self.features):
                out[feature]=content[i][feature]
            return out

        def parseT(self,decodedPacket):
            ''' Parse data from a T packet'''
            content=decodedPacket.decode('ascii')
            return content

        def parseE(self,decodedPacket):
            ''' Parse data from a T packet'''
            content=decodedPacket.decode('ascii')
            return content


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
            sleep(0.1)
            self.port.close()

        def sendCommand(self,command):
            cmd=''.join([command,'\n'])
            self.port.write(str.encode(cmd))
            self.port.flush()

        def _readPort(self):
            ''' Read the serial port and return a fifo list of the packets in the buffer'''
            if self.port.in_waiting:
                a=self.port.read(self.port.in_waiting)
                self.buffer=self.buffer+a
                packets=self.buffer.split(b'\0') #Split by packet end char
                self.buffer=packets[-1]
                packets=packets[0:-1]
                #Read each packet using cobs decode
                decoded=[]
                for packet in packets:
                    try:
                        decoded.append(cobs.decode(packet))
                    except Exception as ex:
                        print('Problem interpreting packet:')
                        print(ex)
                        print(packet)
                        continue
                return decoded
            else:
                return []

        def readOrdered(self):
            ''' Read the packets in the port returning a list of tuples with packet type'''
            decoded=self._readPort()
            out=[]
            for d in decoded:
                if self.ispython2:
                    a=decoded[0]
                else:
                    a=chr(decoded[0])
                if a in self.packetTypes:
                    out.append((a,self.packetFunctions[a](d)))
            return out

        def read(self):
            ''' Read the packets in the port returning a dictionary with the processed data'''
            data=self.read_raw()
            for typeIdx,packetType in enumerate(self.packetTypes):
                for i,d in enumerate(data[packetType]):
                    data[packetType][i]=self.packetFunctions[typeIdx](d)

            return data

        def read_raw(self):
            ''' Read the packets in the port returning a dictionary with a list
            of raw data packets for each packet type'''
            decoded=self._readPort()
            data=self.emptyPackets()
            for d in decoded:
                if self.ispython2:
                    a=d[0]
                else:
                    a=chr(d[0])
                for packetType in self.packetTypes:
                    if a==packetType:
                            data[a].append(d[1:])
            return data

        def readold(self):
            '''Read serial the data to the RX buffer and use processing functions for
            each packet type. Returns the processing results for each packet type as a
            dictionary.
            '''
            data=self._emptyPacket.copy()

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
                elif a=='E':
                    print(decoded[1:])
                    data['E'].append(decoded[1:].decode('ascii'))
                else:
                    print('Unsupported Packet type:'),
                    print(decoded)
            return data

        def _readString(self):
            #Read whatever is in in serial as string and clear the buffer
            a=self.port.read(self.port.in_waiting)
            self.buffer=self.buffer+a
            packets=self.buffer.split(b'\0')
            self.buffer=b''
            return packets[-1]
