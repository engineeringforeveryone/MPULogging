# serial.py

import serial
import keyboard

import sys
import glob
import serial

import threading
import datetime

# https://stackoverflow.com/questions/12090503/listing-available-com-ports-with-python
def serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)

        except (OSError, serial.SerialException):
            pass
    return result

# ============================================================================ #

import os

def write_file(data):
    filename =  datetime.datetime.now().strftime('%Y%m%d_%H%M%S') + '.csv'
    filepath = 'out/' + filename

    count = 1
    with open(filepath, 'w') as outfile:
        outfile.write(',ax,ay,az,gx,gy,gz,cx,cy,cz\n')
        for line in data:
            outfile.write(str(count) + ','+line)
        
    print('Successfully wrote to file: ' + filename)


# data from Arduino is in the format
# accelerometer, gyroscope, compass
# ax, ay, az, gx, gy, gz, cx, cy, cz

# run this script after the arduino is connected
# press q to quit.
# 
def main():
    available_ports = serial_ports()

    # print(available_ports)

    # connect to serial port
    # for now serial port is COM10 or second port returned by serial_ports()
    logger_port = 'COM10'
    if len(available_ports) > 1:
        logger_port = available_ports[1]
    else:
        print('Arduino Micro not connected. Exiting...')
        exit()


    print('Started logging serial data. ')    

    # set up serial
    ser = serial.Serial()
    ser.baudrate = 9600
    ser.port = logger_port
    ser.open()

    
    if ser.is_open:
        print('\nLogging...')
        
        data_buffer = list()
        count = 0
        while True:
                
            in_line = ser.readline().decode('utf-8')
            data_buffer.append(in_line)

            # 64,15796,-4808,-300,21,-85,20,610,-48

            count += 1
            # quit if q is pressed
            if keyboard.is_pressed('q'):
                print('Quitting')
                ser.close()
                break

            if count % 2500 == 0:
                thread_writefile = threading.Thread(target=write_file, args=(data_buffer.copy(), ))
                thread_writefile.start()
                print('2500 lines read.')
                
                # clear buffer
                data_buffer = []
                
                
            
        print(count, 'lines read.')

if __name__=='__main__':
    main()
