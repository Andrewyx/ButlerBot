import serial
from time import sleep

ser = serial.Serial ("/dev/ttyS0", 9600)    #Open port with baud rate
while True:
    #received_data = ser.read()              #read serial port
    #sleep(0.03)
    #data_left = ser.inWaiting()             #check for remaining byte
    #received_data += ser.read(data_left)

    #print(received_data)                   #print received data

    ser.write(b"123445678\n")
    print("0") 
    sleep(1)  
    ser.write(b"1\n")
    print("1")
    sleep(1)
