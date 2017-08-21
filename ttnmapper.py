#
# ttnmapper.py
#
# Implementation of a ttnmapper-node for LoPy.
# See http://ttnmapper.org and https://www.pycom.io/product/lopy for details.
#
# Copyright (C) 2017, Peter Affolter and Pascal Mainini
# Licensed under MIT license, see included file LICENSE or
# http://opensource.org/licenses/MIT
#

import array
import pycom
import socket
import time

from binascii import hexlify, unhexlify
from machine import Pin, UART, Timer
from network import LoRa

from L76GNSS import L76GNSS
from pytrack import Pytrack


################################################################################
# Configuration and Constants
################################################################################

# LoRaWAN Configuration
dev_eui = unhexlify('00B23A6A5892A055'.replace(' ',''))
app_eui = unhexlify('70B3D57EF0006815'.replace(' ',''))
app_key = unhexlify('BC6F6A2E7C2CA2B6B5DE7B3506E4AAF1'.replace(' ',''))

# Interval between measures transmitted to TTN.
# Measured airtime of transmission is 56.6 ms, fair use policy limits us to
# 30 seconds per day (= roughly 500 messages). We default to a 180 second
# interval (=480 messages / day).
SEND_RATE       = 60 #180

# GNSS Configuration
GNSS_TIMEOUT    = 12000      # Timeout for obtaining position (miliseconds)

# Colors used for status LED
RGB_OFF         = 0x000000
RGB_POS_UPDATE  = 0x403000
RGB_POS_FOUND   = 0x004000
RGB_POS_NFOUND  = 0x400000
RGB_LORA_JOIN   = 0x000040
RGB_LORA_JOINED = 0x004000
LED_TIMEOUT     = 0.2

sock = 0

################################################################################
# Function Definitions
################################################################################

def log(msg):
    """Helper method for logging messages"""
    print('ttnmapper: {}'.format(msg))

def init_lora():
    """Initialize LoRaWAN connection"""

    lora = LoRa(mode=LoRa.LORAWAN)
    log('Initializing LoRaWAN, DEV EUI: {} ...'.format(hexlify(lora.mac())
        .decode('ascii').upper()))

    if not app_key:
        log('ERROR: LoRaWAN APP KEY not set!')
        log('Send your DEV EUI to thethingsnetwork@bfh.ch to obtain one.')
        return (None, None)

    pycom.rgbled(RGB_LORA_JOIN)

    lora.join(activation=LoRa.OTAA, auth=(dev_eui, app_eui, app_key), timeout=0)

    while not lora.has_joined():
        log('Joining...')
        pycom.rgbled(RGB_OFF)
        time.sleep(LED_TIMEOUT)
        pycom.rgbled(RGB_LORA_JOIN)
        time.sleep(2.5)

    pycom.rgbled(RGB_OFF)

    # Setup socket
    sock = socket.socket(socket.AF_LORA, socket.SOCK_RAW)
    sock.setsockopt(socket.SOL_LORA, socket.SO_DR, 5)      # Set data rate
    sock.setblocking(False)

    log('Done!')
    return (lora, sock)

def gnss_position():
    """Obtain current GNSS position.
    If a position has been obtained, returns an instance of NmeaParser
    containing the data. Otherwise, returns None."""
    start = time.ticks_ms()
    count = 0
    while count < 20:
        coord = l76.coordinates()
        count += 1
    while time.ticks_diff(start, time.ticks_ms()) < GNSS_TIMEOUT:
        coord = l76.coordinates()
        if coord[0] != 0 and coord[2] != 0:
            log('Current position: {}'.format(coord))
            return coord

    log('No position')
    return None

def transmit(pos):
    """Encode current position, altitude and hdop and send it using LoRaWAN"""

    data = array.array('B', [0, 0, 0, 0, 0, 0, 0, 0, 0])

    lat = int(((pos[0] + 90) / 180) * 16777215)
    print(lat)
    data[0] = (lat >> 16) & 0xff
    data[1] = (lat >> 8) & 0xff
    data[2] = lat & 0xff

    lon = int(((pos[1] + 180) / 360) * 16777215)
    print(lon)
    data[3] = (lon >> 16) & 0xff
    data[4] = (lon >> 8) & 0xff
    data[5] = lon &0xff

    alt = int(pos[3])
    print(alt)
    data[6] = (alt >> 8) & 0xff
    data[7] = alt & 0xff

    hdop = int(pos[2] * 10)
    print(hdop)
    data[8] = hdop & 0xff

    message = bytes(data)
    count = sock.send(message)

    log('Message sent: {} ({} bytes)'.format(hexlify(message).upper(), count))

def update_task(alarmtrigger):
    """Periodically run task which tries to get current position and update
       ttnmapper"""

    pycom.rgbled(RGB_POS_UPDATE)
    time.sleep(LED_TIMEOUT)
    pos = gnss_position()

    if pos:
        pycom.rgbled(RGB_POS_FOUND)
        transmit(pos)
    else:
        pycom.rgbled(RGB_POS_NFOUND)

    time.sleep(LED_TIMEOUT)
    pycom.rgbled(RGB_OFF)

    machine.idle()


################################################################################
# Main Program
################################################################################

log('Starting up...')

pycom.heartbeat(False)      # Turn off hearbeat LED

py = Pytrack()
l76 = L76GNSS(py, timeout=30)

(lora, sock) = init_lora()

if lora:
    mapper = Timer.Alarm(update_task, s=SEND_RATE, periodic=True)

log('Startup completed')
