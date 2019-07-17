# implementation of Light algorithm (gateway side)
# "Offline scheduling algorithms for time-slotted lora-based bulk data transmission"
# author: Dimitris Zorbas (dimzorbas@ieee.org)
#
# Distributed under GNU GPLv3

import socket
import struct
from network import LoRa
from network import WLAN
import ubinascii
import pycom
import time
import uos
from machine import Timer
import math

pycom.heartbeat(False)
off = 0x000000
red = 0x7f0000
green = 0x007f00
blue = 0x00007f
white = 0xFFFAFA

wlan = WLAN(mode=WLAN.STA)
if not wlan.isconnected():
    wlan.connect('ssid', auth=(WLAN.WPA2, 'password'), timeout=5000)
    while not wlan.isconnected():
        machine.idle()

print (ubinascii.hexlify(wlan.mac(),':').decode())
print("I got IP"+wlan.ifconfig()[0])

_LORA_PKG_FORMAT = "!BB%ds"
_LORA_INIT_FORMAT = "!BBs"
_LORA_RCV_PKG_FORMAT = "!BB%ds"
MY_ID = 0x01
my_sf = int(MY_ID) + 6
my_bw_index = 2
(guard, sync_method, sync_rate) = (40, 1, 1)
freqs = [865000000, 865600000, 866200000, 866800000, 867400000, 868000000] # my channels
airtime = [[0.174336, 0.087168, 0.043584], [0.307712, 0.153856, 0.076928], [0.553984, 0.276992, 0.138496], [1.026048, 0.513024, 0.256512], [2.215936, 0.944128, 0.472064], [3.940352, 1.724416, 0.862208]]
if (my_bw_index == 0):
    my_bw = LoRa.BW_125KHZ
elif (my_bw_index == 1):
	my_bw = LoRa.BW_250KHZ
elif (my_bw_index == 2):
    my_bw = LoRa.BW_500KHZ
my_node_list = [11, 12, 13, 14, 15, 16, 17, 18, 19, 20] # my node ids

while (True):
    # connect to the network server via wifi and receive the schedule
    pycom.rgbled(green)
    host = '192.168.0.'+str(int(MY_ID))
    port = 8000
    wlan_s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print("socket created")
    wlan_s.bind((host, port))
    wlan_s.listen(5)
    conn, addr = wlan_s.accept()
    print('Got connection from', addr)
    data = conn.recv(512)
    data = str(data)[2:]
    data = data[:-1]
    if (len(data) > 30): # get some info + the schedule
        (guard, sync_method, sync_rate, data) = str(data).split(":")
        data = str(guard)+":"+str(sync_rate)+":"+str(data) # send in sting mode (I'll change this later)
        print(data)
    wlan_s.close()
    guard = int(guard)
    sync_method = int(sync_method)
    sync_rate = int(sync_rate)
    airt = airtime[my_sf-7][my_bw_index]*1000
    round_length = int(math.ceil(100*airt/(airt + 2*guard))*(airt + 2*guard))
    print("round length =", round_length)

    # send the schedule to the nodes (let's use SF12)
    print("start with lora...")
    lora = LoRa(mode=LoRa.LORA, tx_iq=True, frequency=freqs[0], region=LoRa.EU868, power_mode=LoRa.ALWAYS_ON, bandwidth=my_bw, sf=12, tx_power=14)
    lora_sock = socket.socket(socket.AF_LORA, socket.SOCK_RAW)
    lora_sock.setblocking(False)
    pkg = struct.pack(_LORA_PKG_FORMAT % len(data), MY_ID, len(data), data)
    print(pkg)
    while (lora.ischannel_free(-100) == False):
        time.sleep_ms(100)
    lora_sock.send(pkg)
    print("schedule sent!")
    time.sleep(1)
    while (lora.ischannel_free(-100) == False):
        time.sleep_ms(100)
    data = "init"
    pkg = struct.pack(_LORA_PKG_FORMAT % len(data), MY_ID, len(data), data)
    lora_sock.send(pkg)
    print("Init command sent!")
    my_data = list()
    time.sleep_ms(207) # propagation time of init with SF12
    chrono = Timer.Chrono()
    chrono.start()
    start = chrono.read_ms()
    finish = start
    i = 1
    while ((finish - start) < (round_length*102 + 100*(3*guard + 50)/sync_rate)): # data collection time + some extra
        pycom.rgbled(red)
        round_start = chrono.read_ms()
        print(i, "----------------------------------------------------")
        print("started new round at:", round_start)
        lora.init(mode=LoRa.LORA, rx_iq=True, region=LoRa.EU868, frequency=freqs[my_sf-7], power_mode=LoRa.ALWAYS_ON, bandwidth=my_bw, sf=my_sf)
        print("started receiving at:", chrono.read_ms())
        while ((chrono.read_ms() - round_start) < (round_length)):
            recv_pkg = lora_sock.recv(8192)
            if (len(recv_pkg) > 2):
                recv_pkg_len = recv_pkg[1]
                recv_pkg_id = recv_pkg[0]
                if (int(recv_pkg_id) <= 20) and (int(recv_pkg_len) == 98):
                    dev_id, leng, msg = struct.unpack(_LORA_RCV_PKG_FORMAT % recv_pkg_len, recv_pkg)
                    print('Device: %d - Pkg:  %s' % (dev_id, msg))
                    if (str(msg) == "b'11111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111'"): # format check
                        msg = int(dev_id)
                        my_data.append(msg)

        if (i % sync_rate == 0): # synchronisation
            sync_slot = 100 # I have to fix this
            sync_start = chrono.read_ms()
            print("entered sync at:", sync_start)
            pycom.rgbled(white)
            time.sleep_ms(2*guard) # let's make it long so all the nodes are up
            lora.init(mode=LoRa.LORA, tx_iq=True, frequency=freqs[my_sf-7], region=LoRa.EU868, power_mode=LoRa.ALWAYS_ON, bandwidth=my_bw, sf=my_sf, tx_power=14)
            if (sync_method == 1): # 1st sync method
                data = int(chrono.read_ms())
                data = str(data)
            else: # 2nd sync method
                data = "sync"
            pkg = struct.pack(_LORA_PKG_FORMAT % len(data), MY_ID, len(data), data)
            while (lora.ischannel_free(-100) == False): # precision killer
                print("act on channel!")
                time.sleep_ms(100)
                if (sync_method == 1):
                    data = int(chrono.read_ms())
                    data = str(data)
                pkg = struct.pack(_LORA_PKG_FORMAT % len(data), MY_ID, len(data), data)
            lora_sock.send(pkg)
            print("Sent sync: "+data)
            time.sleep_ms(guard)
            print("sync lasted:", abs(time.ticks_diff(int(chrono.read_ms()), int(sync_start))), "ms")

        finish = chrono.read_ms()
        print("round lasted:", abs(time.ticks_diff(int(finish), int(round_start))), "ms")
        i += 1

    time.sleep(1)
    pycom.rgbled(green)
    occur = {}
    stats = []
    for n in my_node_list:
        occur[n] = 0
    for n in my_data:
        occur[n] += 1
    for n in sorted(occur.keys()):
        print("%s: %s" % (n, occur[n]))
        stats.append(occur[n])
    print(str(stats))
    # send data to network server
    wlan_s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    wlan_s.connect(('192.168.0.254', 8000))
    wlan_s.send(str(stats))
    wlan_s.close()
    print("data was sent to network server!")
    pycom.rgbled(off)
    time.sleep(10)
