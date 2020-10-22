import smbus  # import SMBus module of I2C
from time import sleep  # import
import time
import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
# Logging for tests
import logging
import os
# Bluetooth thread
import bluetooth
from threading import Thread
import socket
import subprocess


global PWR_MGMT_1
global SMPLRT_DIV
global CONFIG
global GYRO_CONFIG
global INT_ENABLE
global ACCEL_XOUT_H
global ACCEL_YOUT_H
global ACCEL_ZOUT_H
global GYRO_XOUT_H
global GYRO_YOUT_H
global GYRO_ZOUT_H
# some MPU6050 Registers and their Address
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

global isLogging
global logFileName
global messageNumber
global acc_x
global acc_y
global acc_z
global gyro_x
global gyro_y
global gyro_z
global acc_x2
global acc_y2
global acc_z2
global gyro_x2
global gyro_y2
global gyro_z2
global bus

global Ax
global Ay
global Az
global Gx
global Gy
global Gz

global Device_Address1
global Device_Address2
global value1
global value2

global newDataFlag
global bluetoothConnected

global socketToHost

isLogging = True

logFileName = ""
messageNumber = 0
acc_x = 0
acc_y = 0
acc_z = 0
gyro_x = 0
gyro_y = 0
gyro_z = 0

Ax = 0
Ay = 0
Az = 0
Gx = 0
Gy = 0
Gz = 0

acc_x2 = 0
acc_y2 = 0
acc_z2 = 0
gyro_x2 = 0
gyro_y2 = 0
gyro_z2 = 0

Ax2 = 0
Ay2 = 0
Az2 = 0
Gx2 = 0
Gy2 = 0
Gz2 = 0


value1 = 0
value2 = 0

newDataFlag = False
bluetoothConnected = False

class NetworkHandler:
    def __init__(self):
        self._running = True
        print("Wifi Checker Class has been initiated.")

    def terminate(self):
        self._running = False

    def run(self):
        print("Wifi Checker Thread started")

        #
        while True:
            output = subprocess.check_output("ifconfig wlan0|grep inet | awk '{print $2}'", shell=True)
            print("This is my IP : " + output)
            time.sleep(5)

# Not used at this time, bananapi doe not use bluetooth
class BluetoothServer:
    def __init__(self):
        self._running = True

    def terminate(self):
        self._running = False

    def run(self):
        global newDataFlag

        while True:
            print('OPENING socket')
            try:
                server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
                port = 1
                server_sock.bind(("", port))
                server_sock.listen(1)
                client_sock, address = server_sock.accept()

                print('Accepted connection from ', address)
                bluetoothConnected = True
                #led_red.on()
                #led_green.off()
                while self._running:
                    try:
                        # data = client_sock.recv(1024)
                        # print ("received [%s]" % data)

                        if (newDataFlag == True):
                            try:
                                newDataFlag = False
                                print('There are new Data -> Send to Bluetooth')
                                client_sock.sendall(''.join(format(x, '02x') for x in transportDataGroup))
                                client_sock.sendall("\r\n")
                            except Exception as e:
                                newDataFlag = False
                                print(e)
                                print('Enable to send')
                                # Exception saving to the log file
                                writeLogFileError("Exception while send data to bluetooth!")
                                break

                    except:
                        print('Enable to receive')
                        writeLogFileError("Exception disable the bluetooth communication")
                        break

                print("Closing socket")
                client_sock.close()
                server_sock.close()
                bluetoothConnected = False

            except:
                writeLogFileError("Exception while connection to the bluetooth")
                print("Closing socket")
                client_sock.close()
                server_sock.close()
                bluetoothConnected = False


def initSocket():
    global socketToHost
    HOST = '192.168.1.2'
    PORT = 6000
    socketToHost = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    socketToHost.connect((HOST, PORT))

def MPU_Init():
    global Device_Address1
    global Device_Address2
    global PWR_MGMT_1
    global SMPLRT_DIV
    global CONFIG
    global GYRO_CONFIG
    global INT_ENABLE
    global ACCEL_XOUT_H
    global ACCEL_YOUT_H
    global ACCEL_ZOUT_H
    global GYRO_XOUT_H
    global GYRO_YOUT_H
    global GYRO_ZOUT_H

    print("Bus: " + str(bus) + " Device Address: " + str(Device_Address1) + " Sample rate register: " + str(SMPLRT_DIV))

    # write to sample rate register
    bus.write_byte_data(Device_Address1, SMPLRT_DIV, 2)

    # Write to power management register
    bus.write_byte_data(Device_Address1, PWR_MGMT_1, 1)

    # Write to Configuration register
    bus.write_byte_data(Device_Address1, CONFIG, 0)

    # Write to Gyro configuration register
    bus.write_byte_data(Device_Address1, GYRO_CONFIG, 24)

    # Write to interrupt enable register
    bus.write_byte_data(Device_Address1, INT_ENABLE, 1)

    # write to sample rate register
    bus.write_byte_data(Device_Address2, SMPLRT_DIV, 2)

    # Write to power management register
    bus.write_byte_data(Device_Address2, PWR_MGMT_1, 1)

    # Write to Configuration register
    bus.write_byte_data(Device_Address2, CONFIG, 0)

    # Write to Gyro configuration register
    bus.write_byte_data(Device_Address2, GYRO_CONFIG, 24)

    # Write to interrupt enable register
    bus.write_byte_data(Device_Address2, INT_ENABLE, 1)


def read_raw_data(addr):
    global Device_Address1
    global Device_Address2
    global value1
    global value2
    # Accelero and Gyro value are 16-bit
    high1 = bus.read_byte_data(Device_Address1, addr)
    low1 = bus.read_byte_data(Device_Address1, addr + 1)

    high2 = bus.read_byte_data(Device_Address2, addr)
    low2 = bus.read_byte_data(Device_Address2, addr + 1)

    # concatenate higher and lower value
    value1 = ((high1 << 8) | low1)
    value2 = ((high2 << 8) | low2)

    # to get signed value from mpu6050
    if (value1 > 32768):
        value1 = value1 - 65536

    if (value2 > 32768):
        value2 = value2 - 65536

def initNewLogFile():
    global logFileName
    # init logging
    LOG_LEVEL = logging.INFO
    # LOG_LEVEL = logging.DEBUG
    setNextLogFileNumber()
    # LOG_FILE = "/dev/stdout"
    LOG_FORMAT = "%(asctime)s %(levelname)s %(message)s"
    logging.basicConfig(filename=logFileName, format=LOG_FORMAT, level=LOG_LEVEL)

def closeLogFile():
    logging.shutdown()

def setNextLogFileNumber():
    global logFileName
    onlyfiles = [f for f in os.listdir("/home/pi") if os.path.isfile(os.path.join("/home/pi", f))]
    stringNumber = 0
    # print("Find files in the directiory:")
    # print("Original file list: " + str(onlyfiles))
    if len(onlyfiles) != 0:
        numbersList = [0]
        for fileItem in onlyfiles:
            if fileItem.find("accplot_for_testing_") >= 0:
                numbersList.append(fileItem[20:len(fileItem)])
                print(fileItem[20:len(fileItem)])
        # print("Maximum: " + str(max(numbersList)))
        if len(numbersList) != 0:
            stringNumber = str(int(max(numbersList)) + 1)
        logFileName = "%s%s" % ("accplot_for_testing_", stringNumber)
    else:
        logFileName = "accplot_for_testing_0"

def writeLogFileITEM(msg):
    global isLogging
    if isLogging:
        logging.info(msg)

def writeINFOLogFile(msg):
    global isLogging
    if isLogging:
        writeLogFileITEM("INFO;" + str(int(time.time())) + ";" + str(messageNumber) + ";" + msg)

def writeGPSLogInfos():
    global dateTimeYear
    global dateTimeMonth
    global dateTimeDay
    global dateTimeHour
    global dateTimeMin
    global dateTimeSec
    global lat
    global lon
    global latTarget
    global lonTarget
    global altitude
    global speed
    global isLogging

    global messageNumber

    if isLogging:
        writeLogFileITEM("GPS;" + str(int(time.time())) + ";" + str(
            messageNumber) + ";" + dateTimeYear + "." + dateTimeMonth + "." + dateTimeDay + ";" + dateTimeHour + ":" + dateTimeMin + ":" + dateTimeSec)
        messageNumber += 1

def writeDATELogInfos():

    global messageNumber
    global isLogging

    global acc_x
    global acc_y
    global acc_z

    global gyro_x
    global gyro_y
    global gyro_z

    global Ax
    global Ay
    global Az

    global Gx
    global Gy
    global Gz

    global acc_x2
    global acc_y2
    global acc_z2

    global gyro_x2
    global gyro_y2
    global gyro_z2

    global Ax2
    global Ay2
    global Az2

    global Gx2
    global Gy2
    global Gz2

    ### print('\n\rTry to write to socket ...\n\r')
    ### socketToHost.send("Test")
    if isLogging == True:
        resultstring = "DATA1;" + str(int(time.time())) + ";" + str(messageNumber) + ";" + str(acc_x) + ";" + str(acc_y) + ";" + str(acc_z) + ";" + str(gyro_x) + ";" + str(gyro_y) + ";" + str(gyro_z) + ";" + str(Ax) + ";" + str(Ay) + ";" + str(Az) + ";" + str(Gx) + ";" + str(Gy) + ";" + str(Gy) +  ";"  + str(acc_x2) + ";" + str(acc_y2) + ";" + str(acc_z2) + ";" + str(gyro_x2) + ";" + str(gyro_y2) + ";" + str(gyro_z2) + ";" + str(Ax2) + ";" + str(Ay2) + ";" + str(Az2) + ";" + str(Gx2) + ";" + str(Gy2) + ";" + str(Gy2) + "\r\n"
        socketToHost.send(resultstring.encode())
        writeLogFileITEM(resultstring)
        messageNumber += 1



def writeLogFileError(msg):
    global isLogging
    if isLogging:
        writeLogFileITEM("ERROR;" + str(int(time.time())) + ";" + str(messageNumber) + ";" + msg)

def animate(i, xs, ys):
    global socketToHost
    global acc_x
    global acc_y
    global acc_z

    global gyro_x
    global gyro_y
    global gyro_z

    global Ax
    global Ay
    global Az

    global Gx
    global Gy
    global Gz

    global acc_x2
    global acc_y2
    global acc_z2

    global gyro_x2
    global gyro_y2
    global gyro_z2

    global Ax2
    global Ay2
    global Az2

    global Gx2
    global Gy2
    global Gz2

    global value1
    global value2

    ###print("i : ", i)
    # Add x and y to lists
    ##    xs.append(dt.datetime.now().strftime('%H:%M:%S.%f'))
    ###xs.append(dt.datetime.now().strftime('%H%M%S'))
    ##	#Read Accelerometer raw value
    read_raw_data(ACCEL_XOUT_H)
    acc_x = value1
    acc_x2 = value2
    read_raw_data(ACCEL_YOUT_H)
    acc_y = value1
    acc_y2 = value2
    read_raw_data(ACCEL_ZOUT_H)
    acc_z = value1
    acc_z2 = value2

    # Read Gyroscope raw value
    read_raw_data(GYRO_XOUT_H)
    gyro_x = value1
    gyro_x2 = value2
    read_raw_data(GYRO_YOUT_H)
    gyro_y = value1
    gyro_y2 = value2
    read_raw_data(GYRO_ZOUT_H)
    gyro_z = value1
    gyro_z2 = value2

    # Full scale range +/- 250 degree/C as per sensitivity scale factor
    Ax = acc_x / 16384.0
    Ay = acc_y / 16384.0
    Az = acc_z / 16384.0

    Gx = gyro_x / 131.0
    Gy = gyro_y / 131.0
    Gz = gyro_z / 131.0

    Ax2 = acc_x2 / 16384.0
    Ay2 = acc_y2 / 16384.0
    Az2 = acc_z2 / 16384.0

    Gx2 = gyro_x2 / 131.0
    Gy2 = gyro_y2 / 131.0
    Gz2 = gyro_z2 / 131.0

    #resultString = "Gx=%.2f" % Gx, u'\u00b0' + "/s", "\tGy=%.2f" % Gy, u'\u00b0' + "/s", "\tGz=%.2f" % Gz, u'\u00b0' + "/s","\tAx=%.2f g" % Ax, "\tAy=%.2f g" % Ay, "\tAz=%.2f g" % Az;

    #print(resultString);
    writeDATELogInfos()  # write processed data to the log

    ###ys.append(Az)  # we are plotting Ax timebeing
    sleep(0.002)

    # Limit x and y lists to 40 items

    ###xs = xs[-40:]
    ###ys = ys[-40:]

    ###print("ys: ", ys)
    ###print("xs : ", xs)
    # Draw x and y lists
    ###ax.clear()
    ###ax.plot(xs, ys)

    # Format plot
    ###print("Format plot")
    ###plt.xticks(rotation=45, ha='right')
    ###plt.subplots_adjust(bottom=0.20)

    ###plt.title('acceleration data over Time')
    ###plt.ylabel('m/s')


# Set the Blutooth ON
os.system('sudo systemctl start bluetooth')
os.system('sudo hciconfig hci0 piscan')

# Init classes

# Create Class
BTserver = BluetoothServer()
# Create Thread
BTserverThread = Thread(target=BTserver.run)
# Start Thread
BTserverThread.start()

# Create Wifi checker Class
WifiChecker = NetworkHandler()
# Create a new thread for the wifi checker
WifiCheckerThread = Thread(target=WifiChecker.run())
# Start Thread
WifiCheckerThread.start()


# def uploadDataBase(self):
# read out all data from HW for all nodes in the system
# Start with init and start logging
initNewLogFile()
isLogging = True
writeINFOLogFile("Logging ON")

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = []
ys = []
bus = smbus.SMBus(0)  # or bus = smbus.SMBus(0) for older version boards
Device_Address1 = 0x68  # MPU6050 device address
Device_Address2 = 0x69  # MPU6050 device address

MPU_Init()

print('Wait for init Socket 5 secs')
initSocket()
time.sleep(1)
print('Wait for init Socket 4 secs')
time.sleep(1)
print('Wait for init Socket 3 secs')
time.sleep(1)
print('Wait for init Socket 2 secs')
time.sleep(1)
print('Wait for init Socket 1 secs')
time.sleep(1)
print('Socket connected (Or not :)')



print(" Reading Data of Gyroscope and Accelerometer")
###ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=1)
###plt.show()
while True:             ###
    animate(0,xs,ys)    ###
    sleep(0.002)        ###