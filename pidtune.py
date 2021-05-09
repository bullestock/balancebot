import argparse
import os
import subprocess
import sys
import threading
import websocket

from _secrets import WIFI_PASS
from websocket._abnf import ABNF

try:
    import thread
except ImportError:
    import _thread as thread
import time

# WebSocket stuff

BATTERY = 3
REQ_SET_PID_PARAMS = 5
RES_SET_PID_PARAMS_ACK = 6
REQ_GET_PID_PARAMS = 7
RES_PID_PARAMS = 8
REQ_SET_GYRO_OFFSETS = 9
RES_SET_GYRO_OFFSETS_FAILURE = 10
RES_SET_GYRO_OFFSETS_SUCCESS = 11
REQ_SAVE_CONFIG = 12
RES_SAVE_CONFIG_FAILURE = 13
RES_SAVE_CONFIG_SUCCESS = 14
Q16_ONE = 65536

def get_q16(ba, start):
    sv = ba[start] + 256*ba[start+1] + 256*256*ba[start+2] + 256*256*256*ba[start+3]
    return sv/Q16_ONE

def set_q16(ba, val):
    sv = int(val*Q16_ONE)
    ba.append(sv & 255)
    ba.append((sv // 256) & 255)
    ba.append((sv // (256*256)) & 255)
    ba.append((sv // (256*256*256)) & 255)

def on_message(ws, message):
    ba = bytearray(message)
    opcode = ba[0]
    #print("WS message: %s" % opcode)
    if opcode == BATTERY:
        bat = (ba[1] + 256*ba[2])/1000.0
        print("Battery %2.1f" % bat)
    elif opcode == RES_PID_PARAMS:
        pid_idx = ba[1]
        cur_params = [ get_q16(ba, 2), get_q16(ba, 6), get_q16(ba, 10) ]
        print("PID%d: P %f I %f D %f" % (pid_idx, cur_params[0], cur_params[1], cur_params[2]))
        if pid_idx == 0:
            ws.cur_params = [ cur_params ]
        if pid_idx == 1:
            ws.cur_params.append(cur_params)
            if ws.state == 'WAIT_GET':
                if not ws.set_params:
                    ws.close()
                else:
                    print("Setting PID parameters")
                    try:
                        bytes = [REQ_SET_PID_PARAMS, ws.pid_index]
                        for idx in range(0, 3):
                            param = ws.cur_params[ws.pid_index][idx]
                            if idx < len(ws.pid_params):
                                p = ws.pid_params[idx]
                                if p != None:
                                    param = p
                            set_q16(bytes, param)
                        #print(bytes)
                        ws.sock.send_binary(bytearray(bytes))
                        ws.state = 'WAIT_SET'
                    except Exception as e:
                        print(e)
            elif ws.state == 'WAIT_GET_VERIFY':
                print('All done')
                ws.close()

    elif opcode == RES_SET_PID_PARAMS_ACK:
        print("Verifying PID parameters")
        ws.sock.send_binary(bytearray([REQ_GET_PID_PARAMS, 0]))
        ws.sock.send_binary(bytearray([REQ_GET_PID_PARAMS, 1]))
        ws.state = 'WAIT_GET_VERIFY'

def on_error(ws, error):
    print(error)

def on_close(ws):
    print("WS closed")
    print("ESP: WS close")

def on_open(ws):
    print("ESP: WebSocket")
    # Steering: '0' <velocity> <turn rate>
    def run(*args):
        ws.state = 'INIT'
        while True:
            time.sleep(0.1)
            if ws.state == 'INIT':
                print("Requesting PID parameters")
                ws.sock.send_binary(bytearray([REQ_GET_PID_PARAMS, 0]))
                ws.sock.send_binary(bytearray([REQ_GET_PID_PARAMS, 1]))
                ws.state = 'WAIT_GET'
    thread.start_new_thread(run, ())

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Tune ESPWay PID parameters.')
    parser.add_argument('--index', type=int, nargs='?', default=0,
                        help='PID index (0 = angle, 1 = velocity')
    parser.add_argument('parameter', nargs='*',
                        help="PID parameter (use '-' to keep existing value")

    args = parser.parse_args()
    if len(args.parameter) > 0:
        if len(args.parameter) > 3:
            print('Error: Too many arguments')
            exit()
    if args.index < 0 or args.index > 1:
        print('Error: Invalid index')
        exit()

    print("Connecting to ESPWay WiFi")

    connected = False
    while not connected:
        os.system('sudo nmcli dev wifi list')
        code = os.system('sudo nmcli device wifi connect ESPway password %s' % WIFI_PASS)
        if code == 0:
            connected = True
        elif code == 2560:
            print("ESP: No network")
        else:
            print("ESP: Error %d" % code)

    print("ESP: Connected")
            
    #websocket.enableTrace(True)
    print("Connecting to ESPWay websocket")
    ws = websocket.WebSocketApp("ws://10.0.0.1/",
                                on_open = on_open,
                                on_message = on_message,
                                on_error = on_error,
                                on_close = on_close)
    ws.set_params = False
    if len(args.parameter) > 0:
        ws.set_params = True
        ws.pid_index = args.index
        params = []
        for e in args.parameter:
            p = None
            if e != '-':
                try:
                    p = float(e)
                except ValueError:
                    print("Error: Bad number '%s'" % e)
                    exit()
            params.append(p)
        ws.pid_params = params
    ws.run_forever()
