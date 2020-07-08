import serial
import time
import cobs.cobs as cobs

ser=serial.Serial(
    port='/dev/ttyACM0',
    parity=serial.PARITY_ODD,
    stopbits=serial.STOPBITS_TWO,
    bytesize=serial.SEVENBITS,
)


ser.close()
ser.open()

print('Start')
buffer=''
time.sleep(2)
if ser.in_waiting:
    a=ser.read(ser.in_waiting)
    packets=a.split('\0') #Split by packet end char
    if not (packets[-1]==''):
        buffer=buffer+(packets[-1])
    else:
        buffer=''
    packets=packets[0:-1]
    #Read each packets
    for packet in packets:
        decoded=cobs.decode(packet)
        #First byte is type of packet:
        # 'D' is for data
        # 'T' is for text
        if decoded[0]=='D':
            print('Data:')
            print(decoded[1:])
        if decoded[0]=='T':
            print('Text:')
            print(decoded[1:])

ser.close()
