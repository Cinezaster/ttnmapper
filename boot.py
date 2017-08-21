from machine import UART
import machine
import os
from network import Bluetooth

uart = UART(0, baudrate=115200)
os.dupterm(uart)

# Turn off Bluetooth
bt = Bluetooth()
bt.deinit()

machine.main('main.py')
