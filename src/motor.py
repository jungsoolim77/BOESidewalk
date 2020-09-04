#!/usr/bin/python
import tkinter as tk
import tkinter.font as font
from enum import Enum

import RPi.GPIO as GPIO
from time import sleep
from threading import Thread

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

class Status(Enum):
    FORWARD = 1
    STOP = 2
    BACKWARD = 3

class Window(tk.Frame):

    def __init__(self, parent):
        super(Window, self).__init__(parent)
        self.direction = Status.STOP
        self.buttonFont = font.Font(family='Helvetica', size=12, weight='bold')
        self.forwardB = tk.Button(self, text="  Forward  ", font=self.buttonFont, command=self.forward)
        self.backwardB = tk.Button(self, text="  Backward  ", font=self.buttonFont, command=self.backward)
        self.stopB = tk.Button(self, text="   Stop   ", font=self.buttonFont, command=self.stop)
        self.fastB = tk.Button(self, text="   fast   ", font=self.buttonFont, command=self.set_fast, state=tk.DISABLED)
        self.slowB = tk.Button(self, text="   slow   ", font=self.buttonFont, command=self.set_slow)
        self.statusL = tk.Label(self, text="Status: ", font=self.buttonFont)
        self.speedL = tk.Label(self, text="Speed", font=self.buttonFont)

        self.statusL.grid(row=0, column=0, columnspan=2)
        self.forwardB.grid(row=1, column=0)
        self.stopB.grid(row=1, column=1)
        self.backwardB.grid(row=1, column=2)
        self.speedL.grid(row=2, column=0)
        self.fastB.grid(row=2, column=1)
        self.slowB.grid(row=2, column=2)

        self.fast = 0.0025
        self.slow = 0.0050
        self.speed = self.fast
        self.init()

    def init(self):
        # init GPIO pins
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(enaLF, GPIO.OUT)
        GPIO.setup(dirLF, GPIO.OUT)
        GPIO.setup(pulLF, GPIO.OUT)
        GPIO.setup(enaLB, GPIO.OUT)
        GPIO.setup(dirLB, GPIO.OUT)
        GPIO.setup(pulLB, GPIO.OUT)
        GPIO.setup(enaRF, GPIO.OUT)
        GPIO.setup(dirRF, GPIO.OUT)
        GPIO.setup(pulRF, GPIO.OUT)
        GPIO.setup(enaRB, GPIO.OUT)
        GPIO.setup(dirRB, GPIO.OUT)
        GPIO.setup(pulRB, GPIO.OUT)

        GPIO.output(enaLF, GPIO.LOW)
        GPIO.output(dirLF, GPIO.LOW)
        GPIO.output(pulLF, GPIO.LOW)
        GPIO.output(enaLB, GPIO.LOW)
        GPIO.output(dirLB, GPIO.LOW)
        GPIO.output(pulLB, GPIO.LOW)
        GPIO.output(enaRF, GPIO.LOW)
        GPIO.output(dirRF, GPIO.LOW)
        GPIO.output(pulRF, GPIO.LOW)
        GPIO.output(enaRB, GPIO.LOW)
        GPIO.output(dirRB, GPIO.LOW)
        GPIO.output(pulRB, GPIO.LOW)

    def __del__(self):
        GPIO.cleanup()

    def set_fast(self):
        self.speed = self.fast
        self.fastB.configure(state=tk.DISABLED)
        self.slowB.configure(state=tk.NORMAL)

    def set_slow(self):
        self.speed = self.slow
        self.fastB.configure(state=tk.NORMAL)
        self.slowB.configure(state=tk.DISABLED)

    def forward(self):
        print("move forward")

    def backward(self):
        print("move backward")

    def stop(self):
        print("move stop")

class SideWalkMotor(Window):
    def __init__(self, parent):
        super(SideWalkMotor, self).__init__(parent)
        # self.thread = Thread(target=self.move)
        # self.thread.start()

    def forward(self):
        self.set_direction(Status.FORWARD)
        self.statusL.configure(text="Status: moving forward ")
        print("forward")
        thread = Thread(target=self.move).start()

    def backward(self):
        self.set_direction(Status.BACKWARD)
        self.statusL.configure(text="Status: moving backward")
        print("backward")
        thread = Thread(target=self.move).start()

    def stop(self):
        self.direction = Status.STOP
        self.statusL.configure(text="Status: stopped")
        print("stop")
        self.disableMotor()

    def move(self):
        while(self.direction != Status.STOP):
            GPIO.output(pulLF, GPIO.HIGH)
            GPIO.output(pulLB, GPIO.HIGH)
            GPIO.output(pulRF, GPIO.HIGH)
            GPIO.output(pulRB, GPIO.HIGH)
            sleep(self.speed)
            GPIO.output(pulLF, GPIO.LOW)
            GPIO.output(pulLB, GPIO.LOW)
            GPIO.output(pulRF, GPIO.LOW)
            GPIO.output(pulRB, GPIO.LOW)
            sleep(self.speed)


    def set_direction(self, direction):
        self.direction = direction
        self.enableMotor()
        if(direction == Status.FORWARD):
            GPIO.output(dirLF, GPIO.HIGH)
            GPIO.output(dirLB, GPIO.HIGH)
            GPIO.output(dirRF, GPIO.LOW)
            GPIO.output(dirRB, GPIO.LOW)
        else:
            GPIO.output(dirLF, GPIO.LOW)
            GPIO.output(dirLB, GPIO.LOW)
            GPIO.output(dirRF, GPIO.HIGH)
            GPIO.output(dirRB, GPIO.HIGH)


    def enableMotor(self):
        GPIO.output(enaLF, GPIO.HIGH)
        GPIO.output(enaLB, GPIO.HIGH)
        GPIO.output(enaRF, GPIO.HIGH)
        GPIO.output(enaRB, GPIO.HIGH)

    def disableMotor(self):
        GPIO.output(enaLF, GPIO.LOW)
        GPIO.output(enaLB, GPIO.LOW)
        GPIO.output(enaRF, GPIO.LOW)
        GPIO.output(enaRB, GPIO.LOW)

if __name__ == "__main__":
    root = tk.Tk()
    root.title("Bureau Of Engineering - Sidewalk Slope Monitoring Motor Move")
    root.geometry("350x200")
    main = SideWalkMotor(root)
    main.pack(fill="both", expand=True)
    root.mainloop()