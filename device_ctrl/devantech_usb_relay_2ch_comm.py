#!/usr/bin/python
# -*- coding: utf-8 -*-

import serial
import time
import getopt
import sys
import argparse

## ***要***：sudo chmod 666 /dev/ttyACM0 
PORT_NAME = '/dev/ttyACM0'
BAUD_RATE = 9600

CMD_GET_VERSION = 0x5A
CMD_GET_STATUSES = 0x5B
CMD_SET_STATUSES = 0x5C
CMD_SET_RELAY_ON_ALL = 0x64
CMD_SET_RELAY_ON_CH1 = 0x65
CMD_SET_RELAY_ON_CH2 = 0x66
CMD_SET_RELAY_OFF_ALL = 0x6E
CMD_SET_RELAY_OFF_CH1 = 0x6F
CMD_SET_RELAY_OFF_CH2 = 0x70


def is_stat_on(val):
    if val == '1' or val == 'on':
        return True
    return False


parser = argparse.ArgumentParser(description='Devantech社製 2チャンネルUSBリレーの制御を行います。')
parser.add_argument('-v', dest="flg_get_version", action='store_true', help='バージョンを出力します。')
parser.add_argument('-s', dest="flg_get_statuses", action='store_true', help='リレーの状態を出力します。オン=1,オフ=0。')
parser.add_argument('-S', dest="flg_get_statuses_str", action='store_true', help='リレーの状態を出力します。オン=on,オフ=off。')
parser.add_argument('-all', dest="ch_all", choices=['1', 'on', '0', 'off'], help='全リレーのオン/オフを変更します。')
parser.add_argument('-1', dest="ch_1", choices=['1', 'on', '0', 'off'], help='1chリレーのオン/オフを変更します。')
parser.add_argument('-2', dest="ch_2", choices=['1', 'on', '0', 'off'], help='2chリレーのオン/オフを変更します。')
try:
    args = parser.parse_args()
except argparse.ArgumentError:
    print("コマンド引数解析エラー")
    sys.exit(0)

cmd = None
cmd_val = None
if args.flg_get_version:
    cmd = CMD_GET_VERSION
elif args.flg_get_statuses or args.flg_get_statuses_str:
    cmd = CMD_GET_STATUSES
elif args.ch_all:
    if is_stat_on(args.ch_all):
        cmd = CMD_SET_RELAY_ON_ALL
    else:
        cmd = CMD_SET_RELAY_OFF_ALL
elif args.ch_1 and args.ch_2:
    cmd = CMD_SET_STATUSES
    cmd_val = 0
    if is_stat_on(args.ch_1):
        cmd_val = cmd_val | 0x01
    if is_stat_on(args.ch_2):
        cmd_val = cmd_val | 0x02
elif args.ch_1 and args.ch_2 is None:
    if is_stat_on(args.ch_1):
        cmd = CMD_SET_RELAY_ON_CH1
    else:
        cmd = CMD_SET_RELAY_OFF_CH1
elif args.ch_1 is None and args.ch_2:
    if is_stat_on(args.ch_2):
        cmd = CMD_SET_RELAY_ON_CH2
    else:
        cmd = CMD_SET_RELAY_OFF_CH2
else:
    parser.print_help()
    sys.exit(-1)

retCode = 0
with serial.Serial(PORT_NAME, BAUD_RATE, timeout=None) as ser:
    if cmd == CMD_SET_STATUSES:
        senddata = [cmd, cmd_val]
    else:
        senddata = [cmd]

    ser.write(senddata)

    if cmd == CMD_GET_VERSION:
        recv_data = ser.read(2)
        if len(recv_data) != 2:
            retCode = 1
            printf("version get failed.")
        else:
            print("mod-id=%d,version=%d" % (ord(recv_data[0]), ord(recv_data[1])))
    elif cmd == CMD_GET_STATUSES:
        recv_data = ser.read(1)
        if len(recv_data) != 1:
            retCode = 1
            printf("relay status get failed.")
        else:
            stat_ch_1 = (ord(recv_data[0]) & 0x1) > 0
            stat_ch_2 = (ord(recv_data[0]) & 0x2) > 0
            if args.flg_get_statuses_str:
                print("%s %s" % ("on" if stat_ch_1 else "off", "on" if stat_ch_2 else "off"))
            else:
                print("%d %d" % (stat_ch_1, stat_ch_2))

sys.exit(retCode)
