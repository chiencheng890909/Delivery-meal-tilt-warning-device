import time
import math
import RPi.GPIO as GPIO
import readchar # input wasd
import threading
import bluetooth
from subprocess import call #call aplay
import smbus
import struct
import requests


Motor_R1_Pin = 16
Motor_R2_Pin = 18
Motor_L1_Pin = 13
Motor_L2_Pin = 11
t = 1

Stop = False
start = False
spill = False


class switch:
    def __init__(self):
        self.ENDPOINT = "things.ubidots.com"
        self.DEVICE_LABEL = "FinalProject"
        self.VARIABLE_LABEL = "reset"
        self.TOKEN = "BBFF-3uSFFmRKC7XPSznxkowObbjAKvaMXI"
        self.DELAY = 0.2  # Delay in seconds
        self.URL = "http://{}/api/v1.6/devices/{}/{}/lv".format(self.ENDPOINT, self.DEVICE_LABEL, self.VARIABLE_LABEL)
        self.HEADERS = {"X-Auth-Token": self.TOKEN, "Content-Type": "application/json"}

    def get_var(self):
        status_code = 400
        while status_code >= 400:
            req = requests.get(url=self.URL, headers=self.HEADERS)
            status_code = req.status_code
            # print(int(float(req.text)))
        return int(float(req.text))


class adxl345:
    def __init__(self):
        self.bus = smbus.SMBus(1)
        self.deviceID = self.bus.read_byte_data(0x53, 0x00)
        print("ID: %x" % self.deviceID)

        self.bus.write_byte_data(0x53, 0x2D, 0x00)
        self.bus.write_byte_data(0x53, 0x2D, 0x08)
        self.bus.write_byte_data(0x53, 0x31, 0x08)

    def detection(self):
            accel = {'x': 0, 'y': 0, 'z': 0}
            data0 = self.bus.read_byte_data(0x53, 0x32)
            data1 = self.bus.read_byte_data(0x53, 0x33)
            xAccl = struct.unpack('<h', bytes([data0, data1]))[0]
            accel['x'] = xAccl / 256

            data0 = self.bus.read_byte_data(0x53, 0x34)
            data1 = self.bus.read_byte_data(0x53, 0x35)
            yAccl = struct.unpack('<h', bytes([data0, data1]))[0]
            accel['y'] = yAccl / 256

            data0 = self.bus.read_byte_data(0x53, 0x36)
            data1 = self.bus.read_byte_data(0x53, 0x37)
            zAccl = struct.unpack('<h', bytes([data0, data1]))[0]
            accel['z'] = zAccl / 256

            # print("\nAx Ay Az: %.3f %.3f %.3f" % (accel['x'], accel['y'], accel['z']))
            return accel


class blue_tooth:
    def __init__(self):
        self.target_address = '00:00:B0:01:ED:F7' # 6C:0D:E1:48:43:E1 
        self.client = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        self.port = 30

    def __del__(self):
        self.client.close()

    def connect(self):
        while True:
            try:
                print("Connecting")
                self.client.connect((self.target_address, self.port))
                print("connected")
                break
            except:
                print("fail to connect")
                time.sleep(1)
                continue

    def wait(self):
        global Stop
        global spill
        while not Stop:
            if spill is True:
                call(["mplayer", "/home/pi/final_project/spill.mp3"])
                spill = False


class car:
    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(Motor_R1_Pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(Motor_R2_Pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(Motor_L1_Pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(Motor_L2_Pin, GPIO.OUT, initial=GPIO.LOW)
        print("Press 'q' to quit...")

    def __del__(self):
        GPIO.cleanup()

    def listen(self):
        while True:
            ch = readchar.readkey()
            if ch == 'w':
                self.forward()
            elif ch == 's':
                self.backward()
            elif ch == 'd':
                self.turnRight()
            elif ch == 'a':
                self.turnLeft()
            elif ch == 'q':
                print("\nQuit")
                break

    def stop(self):
        GPIO.output(Motor_R1_Pin, False)
        GPIO.output(Motor_R2_Pin, False)
        GPIO.output(Motor_L1_Pin, False)
        GPIO.output(Motor_L2_Pin, False)

    def forward(self):
        GPIO.output(Motor_R1_Pin, True)
        GPIO.output(Motor_R2_Pin, False)
        GPIO.output(Motor_L1_Pin, True)
        GPIO.output(Motor_L2_Pin, False)
        time.sleep(0.1)
        self.stop()

    def backward(self):
        GPIO.output(Motor_R1_Pin, False)
        GPIO.output(Motor_R2_Pin, True)
        GPIO.output(Motor_L1_Pin, False)
        GPIO.output(Motor_L2_Pin, True)
        time.sleep(0.1)
        self.stop()

    def turnRight(self):
        GPIO.output(Motor_R1_Pin, False)
        GPIO.output(Motor_R2_Pin, False)
        GPIO.output(Motor_L1_Pin, True)
        GPIO.output(Motor_L2_Pin, False)
        time.sleep(0.1)
        self.stop()

    def turnLeft(self):
        GPIO.output(Motor_R1_Pin, True)
        GPIO.output(Motor_R2_Pin, False)
        GPIO.output(Motor_L1_Pin, False)
        GPIO.output(Motor_L2_Pin, False)
        time.sleep(0.1)
        self.stop()


def sw():
    on_off = switch()
    global Stop
    global start
    while not Stop:
        if on_off.get_var() == 1:
            start = True
        else:
            start = False
        time.sleep(1)


def dec():
    detect = adxl345()
    # Net = record()
    ini_values = detect.detection()
    ini_degree = 0
    rad = 57.29577951
    global Stop
    global start
    global spill
    while not Stop:
        vals = detect.detection()
        if start is False:
            max_axis = 'x'
            the_max = 0
            ini_values = vals
            for key, value in ini_values.items():
                if abs(value) > abs(the_max):
                    max_axis = key
                    the_max = value
            if the_max > 1:
                the_max = 1
            elif the_max < -1:
                the_max = -1
            ini_degree = math.acos(the_max)
            ini_degree = 180 - ini_degree * rad

        if vals[max_axis] > 1:
            vals[max_axis] = 1
        elif vals[max_axis] < -1:
            vals[max_axis] = -1
        degree = math.acos(vals[max_axis])
        degree = 180 - degree * rad

        # print('ini_degree =', ini_degree, 'the_degree =', degree)
        print('degree =', abs(degree - ini_degree))
        if abs(degree - ini_degree) > 25:
            spill = True
        # Net.post_var(vals)
        time.sleep(1)


def blue_con():
    blue = blue_tooth()
    # blue.connect()
    blue.wait()


def car_moving():
    small_car = car()
    small_car.listen()
    global Stop
    Stop = True


swi = threading.Thread(target=sw)
dec = threading.Thread(target=dec)
bluetooth_connect = threading.Thread(target=blue_con)
car_control = threading.Thread(target=car_moving)

swi.start()
dec.start()
bluetooth_connect.start()
car_control.start()

car_control.join()
dec.join()
bluetooth_connect.join()
swi.join()
