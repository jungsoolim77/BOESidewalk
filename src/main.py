#!/usr/bin/python
import tkinter as tk
import tkinter.font as font
import tkinter.ttk as ttk
import datetime as time
from time import sleep
import math
from enum import Enum
from threading import Thread
from functools import partial
# import os

#Data from GPIO
import RPi.GPIO as GPIO
import serial
import smbus

#for csv saving
import csv

# port = "/dev/serial0"
portAccelorometer = "/dev/ttyACM0"
portDigitalLevel = "/dev/ttyS0"

# Register
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c
bus = smbus.SMBus(1)  # bus = smbus.SMBus(0) fuer Revision 1

address = 0x68  # via i2cdetect
g = 16384.0  # 14 bits 2^14 = 16384
bus.write_byte_data(address, power_mgmt_1, 0)

enaLF = 4
enaLB = 19
enaRF = 16
enaRB = 26

dirLF = 27
dirLB = 20
dirRF = 6
dirRB = 13

pulLF = 23
pulLB = 12
pulRF = 5
pulRB = 21

# motor status
class Status(Enum):
    FORWARD = 1
    STOP = 2
    BACKWARD = 3

class MSpeed(Enum):
    FAST = 1
    SLOW = 2

#data collection status
class DStatus(Enum):
    INIT = 1
    START = 2
    STOP = 3
    SAVE = 4

class Window(tk.Frame):

    def __init__(self, parent):
        super(Window, self).__init__(parent)
        self.on = False
        self.lineCounter = 0
        self.MAX_LINE = 98
        self.titleFont = font.Font(family='Helvetica', size=12, weight='bold')
        self.labelFont = font.Font(family='Helvetica', size=10, underline=1 )
        self.statusB = tk.Button(self, text="Init GPS", font=self.titleFont,command=partial(self.set_data_collection, DStatus.INIT))
        self.statusB.grid(row=0,column=0,columnspan=2)
        self.startB = tk.Button(self,text="Start",state=tk.DISABLED,font=self.titleFont,width=15,bg='#32a852',fg='#ffffff',command=partial(self.set_data_collection, DStatus.START))
        self.startB.grid(row=0,column=2,columnspan=2)
        self.stopB = tk.Button(self, text="Stop",state=tk.DISABLED,font=self.titleFont, width=15, bg='#32a852', fg='#ffffff',command=partial(self.set_data_collection, DStatus.STOP))
        self.stopB.grid(row=0, column=4, columnspan=2)
        self.saveB = tk.Button(self, text="Save", state=tk.DISABLED, font=self.titleFont, width=15, bg='#32a852', fg='#ffffff', command=partial(self.set_data_collection, DStatus.SAVE))
        self.saveB.grid(row=0, column=6, columnspan=2)
        self.labelTitle=["    Time Stamp                          ", "             Latitude                 ", "             Logitude            ", "   X Slope    ", "  Y Slope    "]
        for i in range(len(self.labelTitle)):
            (tk.Label(self, text=self.labelTitle[i], font=self.labelFont)).grid(row=1,column=i)

        self.textWindow = tk.Text(self)
        self.textWindow.grid(row=2,column=0,columnspan=7,padx=5, pady=2,sticky="nsew")
        self.scrollb = ttk.Scrollbar(self, takefocus=1, command=self.textWindow.yview)
        self.scrollb.grid(row=2, column=8, sticky='nsew')
        self.textWindow['yscrollcommand'] = self.scrollb.set
        self.textGPS = tk.Text(self)
        #   for data
        self.gpsData = ""
        self.lat = 0.0
        self.dirLat = ""
        self.lon = 0.0
        self.dirLon = ""
        self.speed = 0
        self.trCourse = ""
        self.gpsDT =""
        self.x_a = ""
        self.y_a = ""
        self.x_d = ""
        self.y_d = ""
        #     for motor
        self.direction = Status.STOP
        self.buttonFont = font.Font(family='Helvetica', size=12, weight='bold')
        self.forwardB = tk.Button(self, text="  Forward  ", font=self.buttonFont, command=partial(self.set_motor_action, Status.FORWARD))
        self.backwardB = tk.Button(self, text="  Backward  ", font=self.buttonFont, command=partial(self.set_motor_action, Status.BACKWARD))
        self.stopMB = tk.Button(self, text="   Stop   ", font=self.buttonFont, command=partial(self.set_motor_action, Status.STOP))
        self.fastB = tk.Button(self, text="   fast   ", font=self.buttonFont, state=tk.DISABLED, command=partial(self.set_motor_speed, MSpeed.FAST))
        self.slowB = tk.Button(self, text="   slow   ", font=self.buttonFont, command=partial(self.set_motor_speed, MSpeed.SLOW))
        self.statusL = tk.Label(self, text="Status: ", font=self.buttonFont)
        self.speedL = tk.Label(self, text="Speed", font=self.buttonFont)
        self.statusL.grid(row=3, column=0, columnspan=2)
        self.forwardB.grid(row=4, column=0)
        self.stopMB.grid(row=4, column=1)
        self.backwardB.grid(row=4, column=2)
        self.speedL.grid(row=4, column=3)
        self.fastB.grid(row=4, column=4)
        self.slowB.grid(row=4, column=5)
        self.fast = 0.0025
        self.slow = 0.0050
        self.motorspeed = self.fast

    def set_motor_speed(self, speed):
        if(speed == MSpeed.FAST):
            self.motorspeed = self.fast
            self.fastB.configure(state=tk.DISABLED)
            self.slowB.configure(state=tk.NORMAL)
        else:
            self.motorspeed = self.slow
            self.fastB.configure(state=tk.NORMAL)
            self.slowB.configure(state=tk.DISABLED)
        return

    def set_data_collection(self, status):
        return

    def set_motor_action(self, direction):
        return

class SideWalk(Window):
    def __init__(self, parent):
        super(SideWalk, self).__init__(parent)
        self.serialAcc = serial.Serial(portAccelorometer, baudrate=9600, timeout=0.5)
        self.serialDig = serial.Serial(portDigitalLevel, baudrate=9600, timeout=1)
        self.raw_data =[]
        self.init()

    def init(self):
        # init GPIO pins
        GPIO.setmode(GPIO.BCM)
        pins = [enaLF,enaLB,enaRF,enaRB, dirLF, dirLB, dirRF, dirRB, pulLF, pulLB, pulRF, pulRB]
        for pin in pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)

    def __del__(self):
        self.serialAcc.close()
        self.serialDig.close()
        GPIO.cleanup()


    def set_data_collection(self, status):
        if(status == DStatus.INIT):
            self.check_GPS()
            self.statusB.configure(text="Ready", bg='blue', state=tk.DISABLED)
            self.startB.configure(state=tk.NORMAL)
        elif (status == DStatus.START):
            self.startB.configure(state=tk.DISABLED)
            self.stopB.configure(state=tk.NORMAL)
            self.saveB.configure(state=tk.NORMAL)
            self.on = True
            self.get_data()
        elif (status == DStatus.STOP):
            self.startB.configure(state=tk.NORMAL)
            self.stopB.configure(state=tk.DISABLED)
            self.on = False
        else:
            self.save_data()
        return

    def check_GPS(self):
        data = (str(self.serialAcc.readline().decode())).split(",")
        # check for $GPRMC
        if data[0] == "$GPRMC":
            self.update_GPS(data)
        else:
            self.after(1, self.check_GPS())

    def get_data(self):
        if self.lineCounter > self.MAX_LINE:
            self.textWindow.delete('1.0', '12.0')
            self.lineCounter -= 11
        t = self.gpsDT
        t.strftime('%Y-%m-%d %H:%M:%S')
        self.get_XY()
        self.get_DigitalLevel()
        self.textWindow.insert(tk.END, f'\n {str(self.gpsDT), self.lat, self.dirLat, self.lon, self.dirLon, self.x_d, self.y_d} ')
        self.textWindow.see("end")
        self.lineCounter += 1
        self.raw_data.append([self.gpsData, self.gpsDT, self.lat, self.dirLat, self.lon, self.dirLon, self.x_d, self.y_d])
        # sleep(1)
        if self.on:
            self.textWindow.after(100, self.get_data)
            if (self.lineCounter == (self.MAX_LINE - 1)):
                self.textGPS.after(500, self.get_GPS)

    # get digital level data
    def get_DigitalLevel(self):
        data = (str(self.serialDig.read(9600).decode('ascii', 'ignore'))).split("DEF")
        sdata = data[0].split("ABC")
        ddata = sdata[-1].split(" ")
        if len(ddata) >= 9:
            self.x_d = ddata[3]
            self.y_d = ddata[6]


    # get GPS $GPRMC data
    def get_GPS(self):
        data = (str(self.serialAcc.readline().decode())).split(",")
        print(data)
        if data[0] == "$GPRMC":
            self.update_GPS(data)

    def update_GPS(self, data):
        self.lat = self.decode(data[3], self.lat)  # latitude
        self.dirLat = data[4]  # latitude direction N/S
        self.lon = self.decode(data[5], self.lon)  # longitute
        self.dirLon = data[6]  # longitude direction E/W
        self.speed = data[7]  # Speed in knots
        self.trCourse = data[8]  # True course
        self.gpsData = ', '.join(data)
        s = data[9][0:-2] + '20' + data[9][-2:] + ' ' + data[1]
        # get GPS date time in UTC need to subtract 7 hours
        self.gpsDT = time.datetime.strptime(s, '%d%m%Y %H%M%S.00') - time.timedelta(hours=7)
        print(self.gpsData)

    def decode(self, coord, old_value):
        #Converts DDDMM.MMMMM > DD deg MM.MMMMM min
        try:
            x = coord.split(".")
            head = x[0]
            tail = x[1]
            deg = float(head[0:-2])
            min = float(head[-2:] + "." + tail) * (1 / 60)
            return deg + min
        except:
            return old_value


    def get_XY(self):
        # def read_byte(reg):
        #     return self.bus.read_byte_data(self.address, reg)

        def read_word(reg):
            h = bus.read_byte_data(address, reg)
            l = bus.read_byte_data(address, reg + 1)
            value = (h << 8) + l
            return value

        def read_word_2c(reg):
            val = read_word(reg)
            if (val >= 0x8000):
                return -((65535 - val) + 1)
            else:
                return val

        def dist(a, b):
            return math.sqrt((a * a) + (b * b))

        def get_y_rotation(x, y, z):
            radians = math.atan2(x, dist(y, z))
            return -math.degrees(radians)

        def get_x_rotation(x, y, z):
            radians = math.atan2(y, dist(x, z))
            return math.degrees(radians)

        xout = read_word_2c(0x3b)
        yout = read_word_2c(0x3d)
        zout = read_word_2c(0x3f)

        x = xout / g
        y = yout / g
        z = zout / g

        self.x_a = "{0:.4f}".format(get_x_rotation(x, y, z))
        self.y_a = "{0:.4f}".format(get_y_rotation(x, y, z))

    def save_data(self):
        t = time.datetime.today()
        name = "./data/" + t.strftime("%Y_%m_%d_%H_%M_%S") + "slope_data.csv"
        # name = t.strftime("%Y_%m_%d") + "slope_data.csv"
        # print("I am saving...")
        with open(name, 'w', encoding='utf-8') as csvfile:
            writer = csv.writer(csvfile)
            while(len(self.raw_data)> 0):
                writer.writerow(self.raw_data.pop(0))
        return

    def set_motor_action(self, direction):
        if(direction == Status.STOP):
            self.direction = Status.STOP
            self.disableMotor()
        else:
            self.set_direction(direction)
            Thread(target=self.move).start()

    def move(self):
        pins = [pulLF,pulLB,pulRF,pulRB]
        while(self.direction != Status.STOP):
            for pin in pins:
                GPIO.output(pin, GPIO.HIGH)
            sleep(self.motorspeed)
            for pin in pins:
                GPIO.output(pin, GPIO.LOW)
            sleep(self.motorspeed)

    def set_direction(self, direction):
        self.direction = direction
        self.enableMotor()
        leftPins = [dirLF, dirLB]
        rightPins = [dirRF, dirRB]
        if(direction == Status.FORWARD):
            for pin in leftPins:
                GPIO.output(pin, GPIO.HIGH)
            for pin in rightPins:
                GPIO.output(pin, GPIO.LOW)
            self.statusL.configure(text="Status: moving forward ")
        else:
            for pin in leftPins:
                GPIO.output(pin, GPIO.LOW)
            for pin in rightPins:
                GPIO.output(pin, GPIO.HIGH)
            self.statusL.configure(text="Status: moving backward")

    def enableMotor(self):
        pins = [enaLF, enaLB, enaRF, enaRB]
        for x in pins:
            GPIO.output(x, GPIO.HIGH)

    def disableMotor(self):
        pins = [enaLF,enaLB,enaRF,enaRB]
        for x in pins:
            GPIO.output(x, GPIO.LOW)


if __name__ == "__main__":
    root = tk.Tk()
    root.title("Bureau Of Engineering - Sidewalk Slope Monitoring")
    root.geometry("940x500")
    main = SideWalk(root)
    main.pack(fill="both", expand=True)
    root.mainloop()