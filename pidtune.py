import websocket
import os
import sys
import subprocess
import threading

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
    return (ba[start] + 256*ba[start+1] + 256*256*ba[start+2] + 256*256*256*ba[start+3])/Q16_ONE

def on_message(ws, message):
    ba = bytearray(message)
    opcode = ba[0]
    #print("WS message: %s" % opcode)
    if opcode == BATTERY:
        bat = (ba[1] + 256*ba[2])/1000.0
        #!!print("Battery %2.1f" % bat)
    elif opcode == RES_PID_PARAMS:
        pid_idx = ba[1]
        pid_p = get_q16(ba, 2)
        pid_i = get_q16(ba, 6)
        pid_d = get_q16(ba, 10)
        print("PID%d: P %f I %f D %f" % (pid_idx, pid_p, pid_i, pid_d))

def on_error(ws, error):
    print(error)

def on_close(ws):
    print("WS closed")
    print("ESP: WS close")

def on_open(ws):
    print("ESP: WebSocket")
    # Steering: '0' <velocity> <turn rate>
    def run(*args):
        sent_query = False
        i = 0
        while True:
            time.sleep(0.1)
            i = i + 1
            if i > 10:
                if not sent_query:
                    print("Request PID parameters")
                    ws.sock.send_binary(bytearray([REQ_GET_PID_PARAMS, 0]))
                    ws.sock.send_binary(bytearray([REQ_GET_PID_PARAMS, 1]))
                    sent_query = True

    thread.start_new_thread(run, ())

if __name__ == "__main__":
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
                                on_message = on_message,
                                on_error = on_error,
                                on_close = on_close)
    ws.on_open = on_open
    ws.run_forever()
