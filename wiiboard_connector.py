import websocket

from websocket._abnf import ABNF

try:
    import thread
except ImportError:
    import _thread as thread
import time

def on_message(ws, message):
    ba = bytearray(message)
    print("WS message: %s" % ba[0])
    if ba[0] == 3:
        bat = ba[1] + 256*ba[2]
        print("Battery: %f" % (bat/1000.0))
    elif ba[0] == 2:
        tilt_x = ba[3] + 256*ba[4]
        tilt_y = ba[1] + 256*ba[2]
        print("Tilt: %f/%f" % (tilt_x/32768.0, tilt_y/32768.0))


def on_error(ws, error):
    print(error)

def on_close(ws):
    print("WS closed")

def on_open(ws):
    print("WS open")
    # Steering: '0' <velocity> <turn rate>
    def run(*args):
        for i in range(3):
            time.sleep(5)
            ba = bytearray([0, 0, 5])
            print("Steer")
            ws.sock.send_binary(ba)
            time.sleep(5)
        ws.close()
        print("thread terminating...")
    thread.start_new_thread(run, ())

if __name__ == "__main__":
    websocket.enableTrace(True)
    ws = websocket.WebSocketApp("ws://10.0.0.1/",
                              on_message = on_message,
                              on_error = on_error,
                              on_close = on_close)
    ws.on_open = on_open
    ws.run_forever()
