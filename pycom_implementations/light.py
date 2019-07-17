# implementation of Light algorithm (node side)
# "Offline scheduling algorithms for time-slotted lora-based bulk data transmission"
# author: Dimitris Zorbas (dimzorbas@ieee.org)
#
# Distributed under GNU GPLv3

import os
import socket
import time
import struct
from network import LoRa
import pycom
import machine
import ubinascii
from network import WLAN
from network import Bluetooth
from network import Server
from pytrack import Pytrack
import uos
from machine import Timer
import math

wlan = WLAN()
wlan.deinit()
bt = Bluetooth()
bt.deinit()
server = Server()
server.deinit()
py = Pytrack()
ANSELC_ADDR = const(0x18E)
py.poke_memory(ANSELC_ADDR, ~(1 << 7))

_LORA_PKG_FORMAT = "!BB%ds"
_LORA_RCV_PKG_FORMAT = "!BB%ds"
MY_ID = 0x0B
(my_sf, my_bw_index, my_slot, guard, sync_rate) = (7, 2, 0, 40, 1) # default values
freqs = [865000000, 865600000, 866200000, 866800000, 867400000, 868000000] # my channels
# airtimes for 100-byte packets and 8 preamble symbols
airtime = [[0.174336, 0.087168, 0.043584], [0.307712, 0.153856, 0.076928], [0.553984, 0.276992, 0.138496], [1.026048, 0.513024, 0.256512], [2.215936, 0.944128, 0.472064], [3.940352, 1.724416, 0.862208]]
if (my_bw_index == 0):
    my_bw = LoRa.BW_125KHZ
elif (my_bw_index == 1):
    my_bw = LoRa.BW_250KHZ
else:
    my_bw = LoRa.BW_500KHZ

pycom.heartbeat(False)
off = 0x000000
red = 0xFF0000
green = 0x00FF00
blue = 0x0000FF
white = 0xFFFAFA
light_green = 0x7CFC00

while (True): # run multiple tests
    pycom.rgbled(green)
    lora = LoRa(mode=LoRa.LORA, rx_iq=True, region=LoRa.EU868, frequency=freqs[0], power_mode=LoRa.ALWAYS_ON, bandwidth=my_bw, sf=12)
    lora_sock = socket.socket(socket.AF_LORA, socket.SOCK_RAW)
    lora_sock.setblocking(False)
    rec = 0
    while (rec == 0):
        recv_pkg = lora_sock.recv(4096)
        if (len(recv_pkg) > 2):
            recv_pkg_len = recv_pkg[1]
            recv_pkg_id = recv_pkg[0]
            print(str(recv_pkg))
            if (int(recv_pkg_id) == 1):
                dev_id, leng, data = struct.unpack(_LORA_RCV_PKG_FORMAT % recv_pkg_len, recv_pkg)
                print('Device: %d - Pkg:  %s' % (dev_id, data))
                data = str(data)[2:]
                data = data[:-1]
                if ":" in str(data): # schedule: guard:sync_rate:id1 sf slot,id2 sf slot,...
                    pycom.rgbled(red)
                    (guard, sync_rate, data) = str(data).split(":")
                    guard = int(guard)
                    sync_rate = int(sync_rate)
                    nodes = str(data).split(",")
                    print(nodes)
                    for n in nodes:
                        (id, sf, slot) = str(n).split(" ")
                        if (int(id) == int(MY_ID)):
                            my_sf = int(sf)
                            my_slot = int(slot)
                            print("guard = ", guard, "sf = ", my_sf, "slot = ", my_slot)
                            print("waiting for sync command")
                elif (str(data) == "init"): # data collection initialisation
                    print("init received!")
                    pycom.rgbled(blue)
                    rec = 1
                    lora.init(power_mode=LoRa.SLEEP)
                    chrono = Timer.Chrono()
                    chrono.start()

    msg = "11111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111" # just a 98-byte message
    pycom.rgbled(light_green)
    i = 1
    airt = airtime[my_sf-7][my_bw_index]*1000 # conversion to ms
    round = int(math.ceil(100*airt/(airt + 2*guard))*(airt + 2*guard)) # we assume that the frame size (round) is defined by the duty cycle (need to change this for a higher number of nodes)
    print("round length = ", round)
    packets = 100 # number of packets to send
    active = 0.0
    print("S T A R T")
    avg_desync = 0.0
    syncs = 1
    while(i <= packets):
        print(i, "----------------------------------------------------")
        start = chrono.read_ms()
        print("started new round at:", start)
        t = int(my_slot*(airt + 2*guard) + guard) + int(avg_desync/syncs) # sleep time before transmission
        machine.idle()
        time.sleep_ms(t)
        pycom.rgbled(red)
        on_time = chrono.read_ms()
        lora.init(mode=LoRa.LORA, tx_iq=True, region=LoRa.EU868, frequency=freqs[my_sf-7], power_mode=LoRa.TX_ONLY, bandwidth=my_bw, sf=my_sf, tx_power=7)
        pkg = struct.pack(_LORA_PKG_FORMAT % len(msg), MY_ID, len(msg), msg)
        lora_sock.send(pkg)
        pycom.rgbled(blue)
        print("Message of "+str(len(pkg))+" bytes sent at:", chrono.read_ms())
        lora.power_mode(LoRa.SLEEP)
        # print(lora.stats())
        cur_time = chrono.read_ms()
        active += (cur_time - on_time)
        t = round - int(cur_time - start)
        machine.idle()
        time.sleep_ms(t)
        if (i % sync_rate == 0): # synchronisatio
            syncs += 1
            sync_slot = 100 # I have to fix this
            rec = 0
            lora.init(mode=LoRa.LORA, rx_iq=True, region=LoRa.EU868, frequency=freqs[my_sf-7], power_mode=LoRa.ALWAYS_ON, bandwidth=my_bw, sf=my_sf)
            sync_start = chrono.read_ms()
            print("started sync slot at:", sync_start)
            while (rec == 0):# and ((chrono.read_ms() - sync_start) <= sync_slot):
                machine.idle()
                pycom.rgbled(white)
                desync = 0
                recv_pkg = lora_sock.recv(100)
                if (len(recv_pkg) > 2):
                    recv_pkg_len = recv_pkg[1]
                    recv_pkg_id = recv_pkg[0]
                    if (int(recv_pkg_id) == (my_sf-7+1)):
                        dev_id, leng, s_msg = struct.unpack(_LORA_RCV_PKG_FORMAT % recv_pkg_len, recv_pkg)
                        s_msg = str(s_msg)[2:]
                        s_msg = s_msg[:-1]
                        try: # 1st sync method
                            s_msg = int(s_msg)
                            s_msg += 10 # propagation time
                            desync = s_msg - int(chrono.read_ms())
                            print("desync: "+str(desync)+"ms")
                            avg_desync += desync
                            if (avg_desync < 0):
                                chrono.stop()
                                time.sleep_ms(abs(int(avg_desync)))
                                chrono.start()
                                avg_desync = 0
                            lora.power_mode(LoRa.SLEEP)
                            active += (chrono.read_ms() - sync_start)
                            time.sleep_ms(guard)
                            rec = 1
                        except ValueError: # alternative sync method
                            if (str(s_msg) == "sync"):
                                print("sync received!")
                                lora.power_mode(LoRa.SLEEP)
                                active += (chrono.read_ms() - sync_start)
                                time.sleep_ms(guard-1)
                                rec = 1
            print("sync slot lasted:", abs(time.ticks_diff(int(chrono.read_ms()), int(sync_start))), "ms")
            pycom.rgbled(blue)
        finish = chrono.read_ms()
        print("round lasted:", abs(time.ticks_diff(int(finish), int(start))), "ms")
        print("Current active time", active, "ms")
        i += 1

    print("Total active time", active, "ms")
    pycom.rgbled(off)
    lora_sock.close()
    time.sleep(20)
