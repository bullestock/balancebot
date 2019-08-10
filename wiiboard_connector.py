import websocket
import os
import bluetooth
import sys
import threading

from websocket._abnf import ABNF

try:
    import thread
except ImportError:
    import _thread as thread
import time

# --------- User Settings ---------
WEIGHT_SAMPLES = 5
# ---------------------------------

# Wiiboard Parameters
CONTINUOUS_REPORTING = "04"  # Easier as string with leading zero
COMMAND_LIGHT = 11
COMMAND_REPORTING = 12
COMMAND_REQUEST_STATUS = 15
COMMAND_REGISTER = 16
COMMAND_READ_REGISTER = 17
INPUT_STATUS = 20
INPUT_READ_DATA = 21
EXTENSION_8BYTES = 32
BUTTON_DOWN_MASK = 8
TOP_RIGHT = 0
BOTTOM_RIGHT = 1
TOP_LEFT = 2
BOTTOM_LEFT = 3
BLUETOOTH_NAME = "Nintendo RVL-WBC-01"


class EventProcessor:
    def __init__(self):
        self.done = False
        self.lock = threading.Lock()
        self.steering = None

    def mass(self, event):
        self.lock.acquire()
        if event.totalWeight > 5:
            left = (event.topLeft+event.bottomLeft)
            right = (event.topRight+event.bottomRight)
            top = (event.topLeft+event.topRight)
            bot = (event.bottomLeft+event.bottomRight)
            SCALE = 5
            lr = SCALE*(left-right)/event.totalWeight
            tb = SCALE*(top-bot)/event.totalWeight

            #print("T %.3f    LR %.3f TB %.3f" % (event.totalWeight, lr, tb))
            self.steering = (lr, tb)
        else:
            self.steering = None
        self.lock.release()

    def get_stering(self):
        self.lock.acquire()
        s = self.steering
        self.lock.release()
        return s

class BoardEvent:
    def __init__(self, topLeft, topRight, bottomLeft, bottomRight, buttonPressed, buttonReleased):

        self.topLeft = topLeft
        self.topRight = topRight
        self.bottomLeft = bottomLeft
        self.bottomRight = bottomRight
        self.buttonPressed = buttonPressed
        self.buttonReleased = buttonReleased
        #convenience value
        self.totalWeight = topLeft + topRight + bottomLeft + bottomRight

class Wiiboard:
    def __init__(self, processor):
        # Sockets and status
        self.receivesocket = None
        self.controlsocket = None

        self.processor = processor
        self.calibration = []
        self.calibrationRequested = False
        self.LED = False
        self.address = None
        self.buttonDown = False
        for i in xrange(3):
            self.calibration.append([])
            for j in xrange(4):
                self.calibration[i].append(10000)  # high dummy value so events with it don't register

        self.status = "Disconnected"
        self.lastEvent = BoardEvent(0, 0, 0, 0, False, False)

        try:
            self.receivesocket = bluetooth.BluetoothSocket(bluetooth.L2CAP)
            self.controlsocket = bluetooth.BluetoothSocket(bluetooth.L2CAP)
        except ValueError:
            raise Exception("Error: Bluetooth not found")

    def isConnected(self):
        return self.status == "Connected"

    # Connect to the Wiiboard at bluetooth address <address>
    def connect(self, address):
        if address is None:
            print "Non existant address"
            return
        self.receivesocket.connect((address, 0x13))
        self.controlsocket.connect((address, 0x11))
        if self.receivesocket and self.controlsocket:
            print "Connected to " + address
            self.status = "Connected"
            self.address = address
            self.calibrate()
            useExt = ["00", COMMAND_REGISTER, "04", "A4", "00", "40", "00"]
            self.send(useExt)
            self.setReportingType()
        else:
            print "Could not connect to Wiiboard at address " + address

    def receive(self):
        while self.status == "Connected" and not self.processor.done:
            data = self.receivesocket.recv(25)
            intype = int(data.encode("hex")[2:4])
            if intype == INPUT_STATUS:
                # TODO: Status input received. It just tells us battery life really
                self.setReportingType()
            elif intype == INPUT_READ_DATA:
                if self.calibrationRequested:
                    packetLength = (int(str(data[4]).encode("hex"), 16) / 16 + 1)
                    self.parseCalibrationResponse(data[7:(7 + packetLength)])

                    if packetLength < 16:
                        self.calibrationRequested = False
            elif intype == EXTENSION_8BYTES:
                self.processor.mass(self.createBoardEvent(data[2:12]))
            else:
                print "ACK to data write received"

    def disconnect(self):
        if self.status == "Connected":
            self.status = "Disconnecting"
            while self.status == "Disconnecting":
                self.wait(100)
        try:
            self.receivesocket.close()
        except:
            pass
        try:
            self.controlsocket.close()
        except:
            pass
        print "WiiBoard disconnected"

    # Try to discover a Wiiboard
    def discover(self):
        print "Press the red sync button on the board now"
        address = None
        bluetoothdevices = bluetooth.discover_devices(duration=6, lookup_names=True)
        for bluetoothdevice in bluetoothdevices:
            if bluetoothdevice[1] == BLUETOOTH_NAME:
                address = bluetoothdevice[0]
                print "Found Wiiboard at address " + address
        if address is None:
            print "No Wiiboards discovered."
        return address

    def createBoardEvent(self, bytes):
        buttonBytes = bytes[0:2]
        bytes = bytes[2:12]
        buttonPressed = False
        buttonReleased = False

        state = (int(buttonBytes[0].encode("hex"), 16) << 8) | int(buttonBytes[1].encode("hex"), 16)
        if state == BUTTON_DOWN_MASK:
            buttonPressed = True
            if not self.buttonDown:
                print "Button pressed"
                self.buttonDown = True

        if not buttonPressed:
            if self.lastEvent.buttonPressed:
                buttonReleased = True
                self.buttonDown = False
                print "Button released"

        rawTR = (int(bytes[0].encode("hex"), 16) << 8) + int(bytes[1].encode("hex"), 16)
        rawBR = (int(bytes[2].encode("hex"), 16) << 8) + int(bytes[3].encode("hex"), 16)
        rawTL = (int(bytes[4].encode("hex"), 16) << 8) + int(bytes[5].encode("hex"), 16)
        rawBL = (int(bytes[6].encode("hex"), 16) << 8) + int(bytes[7].encode("hex"), 16)

        topLeft = self.calcMass(rawTL, TOP_LEFT)
        topRight = self.calcMass(rawTR, TOP_RIGHT)
        bottomLeft = self.calcMass(rawBL, BOTTOM_LEFT)
        bottomRight = self.calcMass(rawBR, BOTTOM_RIGHT)
        boardEvent = BoardEvent(topLeft, topRight, bottomLeft, bottomRight, buttonPressed, buttonReleased)
        return boardEvent

    def calcMass(self, raw, pos):
        val = 0.0
        #calibration[0] is calibration values for 0kg
        #calibration[1] is calibration values for 17kg
        #calibration[2] is calibration values for 34kg
        if raw < self.calibration[0][pos]:
            return val
        elif raw < self.calibration[1][pos]:
            val = 17 * ((raw - self.calibration[0][pos]) / float((self.calibration[1][pos] - self.calibration[0][pos])))
        elif raw > self.calibration[1][pos]:
            val = 17 + 17 * ((raw - self.calibration[1][pos]) / float((self.calibration[2][pos] - self.calibration[1][pos])))

        return val

    def getEvent(self):
        return self.lastEvent

    def getLED(self):
        return self.LED

    def parseCalibrationResponse(self, bytes):
        index = 0
        if len(bytes) == 16:
            for i in xrange(2):
                for j in xrange(4):
                    self.calibration[i][j] = (int(bytes[index].encode("hex"), 16) << 8) + int(bytes[index + 1].encode("hex"), 16)
                    index += 2
        elif len(bytes) < 16:
            for i in xrange(4):
                self.calibration[2][i] = (int(bytes[index].encode("hex"), 16) << 8) + int(bytes[index + 1].encode("hex"), 16)
                index += 2

    # Send <data> to the Wiiboard
    # <data> should be an array of strings, each string representing a single hex byte
    def send(self, data):
        if self.status != "Connected":
            return
        data[0] = "52"

        senddata = ""
        for byte in data:
            byte = str(byte)
            senddata += byte.decode("hex")

        self.controlsocket.send(senddata)

    #Turns the power button LED on if light is True, off if False
    #The board must be connected in order to set the light
    def setLight(self, light):
        if light:
            val = "10"
        else:
            val = "00"

        message = ["00", COMMAND_LIGHT, val]
        self.send(message)
        self.LED = light

    def calibrate(self):
        message = ["00", COMMAND_READ_REGISTER, "04", "A4", "00", "24", "00", "18"]
        self.send(message)
        self.calibrationRequested = True

    def setReportingType(self):
        bytearr = ["00", COMMAND_REPORTING, CONTINUOUS_REPORTING, EXTENSION_8BYTES]
        self.send(bytearr)

    def wait(self, millis):
        time.sleep(millis / 1000.0)

# WebSocket stuff

def on_message(ws, message):
    ba = bytearray(message)
    #print("WS message: %s" % ba[0])
    if ba[0] == 3:
        ws.bat = ba[1] + 256*ba[2]
        #print("Battery: %f" % (bat/1000.0))
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
        while True:
            s = ws.processor.get_stering()
            if s:
                # '0', <turn>, <speed>
                # Range of lr/tb is approx +-2
                lr = s[0]
                turn = int(abs(lr)/2.0*128) / 8
                if turn >= 128:
                    turn = 127
                if lr < 0 and turn > 0:
                    turn = 256-turn
                tb = s[1]
                speed = int(abs(tb)/2.0*128) / 8
                if speed >= 128:
                    speed = 127
                if tb < 0 and speed > 0:
                    speed = 256-speed
                print("Steer %d, %d" % (turn, speed))
                ba = bytearray([0, turn, speed])
                ws.sock.send_binary(ba)
            time.sleep(0.1)
    thread.start_new_thread(run, ())

if __name__ == "__main__":
    processor = EventProcessor()

    board = Wiiboard(processor)

    address = '00:24:44:58:F1:D8'
    
    try:
        # Disconnect already-connected devices.
        # This is basically Linux black magic just to get the thing to work.
        subprocess.check_output(["bluez-test-input", "disconnect", address], stderr=subprocess.STDOUT)
        subprocess.check_output(["bluez-test-input", "disconnect", address], stderr=subprocess.STDOUT)
    except:
        pass

    print "Connecting to Wiiboard..."
    board.connect(address)  # The wii board must be in sync mode at this time
    print "Connected to Wiiboard"
    board.wait(200)
    # Flash the LED so we know we can step on.
    board.setLight(False)
    board.wait(500)
    board.setLight(True)
    wii_thread = threading.Thread(target = board.receive, args=())
    wii_thread.daemon = True
    wii_thread.start()

    print("Connecting to ESPWay WiFi")
    os.system('nmcli connection up 7395620e-a312-4194-af3e-a39f7e038c1e')
    #websocket.enableTrace(True)
    print("Connecting to ESPWay websocket")
    ws = websocket.WebSocketApp("ws://10.0.0.1/",
                                on_message = on_message,
                                on_error = on_error,
                                on_close = on_close)
    ws.processor = processor
    ws.on_open = on_open
    ws.run_forever()
