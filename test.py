# Import section
# Main thread
from threading import Thread
import time
import RPi.GPIO as IO
from gpiozero import LED
from pprint import pprint
import serial
import socket
import select
import binascii

# Bluetooth thread
import bluetooth

# SPI thread
import time
import spidev

# Logging for tests
import logging

import subprocess as sp

# Prepare global variables
global transportData1
global transportData2
global transportData3
global transportData4
global transportDataGroup
global nodeDataBase
global testPackage
global newDataFlag
global uartBTOK
global uartGPSOK
global actRFIDSettings
global sock
global client_sock
global haveSocketOpened
transportData1 = 0
transportData2 = 0
transportData3 = 0
transportData4 = 0
transportDataGroup = []
nodeDataBase = []
testPackage = []
allDataPackage = []
newDataFlag = False
uartBTOK = False
uartGPSOK = False
haveSocketOpened = False

# Prepare SPI variables
# We only have SPI bus 0 available to us on the Pi
bus = 0

# Device is the chip select pin. Set to 0 or 1, depending on the connections
geckoDevice = 0
canDevice = 1

# Enable SPI
spi = spidev.SpiDev()

# Open a connection to a specific bus and device (chip select pin)
spi.open(bus, geckoDevice)

# Set SPI speed and mode
spi.max_speed_hz = 5000000
spi.mode = 0

# Init intou and output ports
IO.setwarnings(False)  # Ignore warning for now
IO.setmode(IO.BCM)
IO.setup(5, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(6, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(12, IO.IN, pull_up_down=IO.PUD_UP)
SIMEnFlag = LED(18)

zummer = LED(13)
led_red = LED(16)
led_green = LED(20)
led_blue = LED(21)
zummer.off()
led_red.off()
led_green.on()
led_blue.on()

# init logging
LOG_LEVEL = logging.INFO
# LOG_LEVEL = logging.DEBUG
LOG_FILE = "hrrLogFile"
# LOG_FILE = "/dev/stdout"
LOG_FORMAT = "%(asctime)s %(levelname)s %(message)s"
logging.basicConfig(filename=LOG_FILE, format=LOG_FORMAT, level=LOG_LEVEL)

# logging.info("Informational message")
# logging.debug("Helpful debugging info")
# logging.error("Something bad happened")

import os

os.system('sudo systemctl start bluetooth')
os.system('sudo hciconfig hci0 piscan')


class BluetoothServer:
    def __init__(self):
        self._running = True

    def terminate(self):
        self._running = False

    def run(self):
        global transportData1
        global transportData2
        global transportData3
        global transportData4
        global transportDataGroup
        global actSettings
        global recSettings
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
                led_red.on()
                led_green.off()
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
                                logging.error("Exception while send data to bluetooth!")
                                break

                    except:
                        print('Enable to receive')
                        logging.error("Exception inenabling the bluetooth communication")
                        break

                print("Closing socket")
                client_sock.close()
                server_sock.close()
                led_red.off()
                led_green.on()

            except:
                logging.error("Exception while connection to the bluetooth")
                print("Closing socket")
                client_sock.close()
                server_sock.close()


class GPSUARTConnection:
    def __init__(self):
        self._running = True

    def terminate(self):
        self._running = False

    def run(self):
        global serGPS
        global uartGPSOK
        global sock
        if uartGPSOK == True:
            try:
                time.sleep(1)
                serialmessage = str.encode('AT\r\n')
                serGPS.write(serialmessage)
                time.sleep(2)
                x = serGPS.readline()
                print(x)
                logging.info("GPS - AT:" + x)
                SocketServerClass.sendSocketMessage(x)
                time.sleep(2)
                x = serGPS.readline()
                print(x)
                logging.info("GPS - AT2:" + x)
                SocketServerClass.sendSocketMessage(x)

                time.sleep(1)
                serialmessage = str.encode('AT+CGNSPWR=1\r\n')
                serGPS.write(serialmessage)
                time.sleep(2)
                x = serGPS.readline()
                print(x)
                logging.info("GPS - AT+CGNSPWR=1:" + x)
                SocketServerClass.sendSocketMessage(x)
                time.sleep(2)
                x = serGPS.readline()
                print(x)
                logging.info("GPS - AT+CGNSPWR2:" + x)
                SocketServerClass.sendSocketMessage(x)

                while True:
                    time.sleep(1)
                    serialmessage = str.encode('AT+CGNSINF\r\n')
                    serGPS.write(serialmessage)
                    time.sleep(2)
                    x = serGPS.readline()
                    print(x)
                    logging.info("GPS - AT+CGNSINF1:" + x)
                    SocketServerClass.sendSocketMessage(x)
                    time.sleep(2)
                    x = serGPS.readline()
                    print(x)
                    logging.info("GPS - AT+CGNSINF2:" + x)
                    SocketServerClass.sendSocketMessage(x)
            except Exception as e:
                print("Error using GPS serial port - " + str(e))
                logging.error("GPS error while using serial port for GPS:" + str(e))
                serGPS.close()
                uartGPSOK = False
        else:
            time.sleep(5)
            self.initGPS()
            self.run()

    def initGPS(self):
        global uartGPSOK
        global serGPS
        # SIMEnFlag.on()
        time.sleep(1)
        print('OPENING serial port for GPS GSN')
        try:
            serGPS = serial.Serial(
                port='/dev/ttyS0',
                baudrate=115200,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1
            )
            uartGPSOK = True
        except:
            # SIMEnFlag.off()
            logging.error("GPS - Could not open Serial port for GPS")
            print("Error opening serial port 0")
        print('GPS GSN connected succesfully')


class SerialPort:
    def __init__(self):
        global ser
        global uartBTOK
        global actRFIDSettings
        global nodeDataBase
        global actSettings
        actRFIDSettings = RfIDSettings()
        self._running = True
        self.initUART()

    def terminate(self):
        self._running = False

    def run(self):
        global ser
        global uartBTOK
        while True:
            if uartBTOK == True:
                try:
                    # print("Write serial port")
                    time.sleep(1)
                    self.readAllByteUART()
                    time.sleep(1)
                    x = ser.readline()
                    self.recParamsAndNodesFromUART(x)
                    # print(x)
                except Exception as e:
                    print("Error using serial port - " + str(e))
                    logging.error("RFID - Error while using serial port for RFiD reader" + str(e))
                    ser.close()
                    uartBTOK = False
            else:
                time.sleep(5)
                self.initUART()

    def readAllByteUART(self):
        global ser
        global uartBTOK
        # print('Read UART')
        CRC = ord('H') ^ ord('R') ^ ord('R') ^ 0x02 ^ 0x00 ^ 0x01 ^ 0x08 ^ ord('M') ^ ord('C') ^ 0
        msg = [ord('H'), ord('R'), ord('R'), 0x02, 0x00, 0x01, 0x08, ord('M'), ord('C'), 0x00, 0x87]
        ser.write(bytes(msg))
        time.sleep(0.00001)
        # print(bytes(msg).hex()+ "\n")
        # print("CRC - " + "0x{:02x}".format(CRC) + " \n")

    def initUART(self):
        global uartBTOK
        global ser
        # print('OPENING serial port 0')
        try:
            ser = serial.Serial(
                port='/dev/ttyUSB0',
                baudrate=9600,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1
            )
            uartBTOK = True
        except:
            logging.error("RFID - error while connecting!")
            try:
                ser = serial.Serial(
                    port='/dev/ttyUSB1',
                    baudrate=9600,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS,
                    timeout=1
                )
                uartBTOK = True
            except:
                logging.error("Error in serial port module: Error opening serial port 0/1 - RFiD")

    def recParamsAndNodesFromUART(self, message):
        global actRFIDSettings
        global nodeDataBase
        global actSettings
        # print('Rec RfID params from message')
        # RfID reader ID
        actRFIDSettings.readerID = message[8:12]
        # print('Reader ID : ' + bytes(actRFIDSettings.readerID).hex() + " \n")
        # Battery of RfID reader voltage
        actRFIDSettings.batteryVoltage = message[12:14]  # 2 bytes
        # print('Battery voltage : ' + bytes(actRFIDSettings.batteryVoltage).hex() + " \n")
        # Temperature of the RfID reader
        actRFIDSettings.temperature = message[14]  # 1 byte
        # print('Device temp: ' + str(actRFIDSettings.temperature) + " \n")
        # Stored positions
        actRFIDSettings.storedPositions = message[15:31]  # 16 Bytes
        testNum = 0
        clientNum = 0
        while testNum < 16:
            if message[15 + testNum] == 0x01:
                clientNum += 1
            testNum += 1
        actSettings.numOfDevices = clientNum
        # print('Stored positions : ' + bytes(actRFIDSettings.storedPositions).hex() + " \n")
        nodeNum = 0
        while nodeNum < 16:
            actRFIDSettings.storedAddresses[nodeNum].ownAddress = message[31 + (nodeNum * 12):43 + (nodeNum * 12)]
            if actRFIDSettings.storedPositions[nodeNum] == 0x01:
                byteNum = 0
                while byteNum < 12:
                    nodeDataBase[nodeNum].txAddress[byteNum] = message[31 + (nodeNum * 12) + byteNum]
                    # print('Address ' + str(nodeDataBase[nodeNum].txAddress[byteNum]) + " \n")
                    byteNum += 1
            nodeNum += 1


class SPIReceiver:
    def __init__(self):
        self._running = True

    def terminate(self):
        self._running = False

    def run(self):
        global transportData1
        global transportData2
        global transportData3
        global transportData4
        global transportDataGroup
        global actSettings
        global testPackage
        global allDataPackage
        global recSettings
        global newDataFlag

        while self._running:
            time.sleep(0.5)

    def sendTestPackage(self):
        print('Read 1 Byte')
        transportData1 = spi.xfer2(testPackage)
        # print("Regcontent [ Node - " + str(nodeAddress) + " Reg - " + "{0:b}".format(regAddress) + " Dummy - " + str(dummyCommand) + "] fist byte for adressing - " + "{0:b}".format(s1Byte)
        print(bytes(transportData1) + "\n")

    def serializeDataBaseToMessage(self):
        global actSettings
        global nodeDataBase
        del allDataPackage[:]
        allDataPackage.append(0x00)  # Read mode
        allDataPackage.append(actSettings.radioMode)  # Command Radio mode
        allDataPackage.append(actSettings.deviceReset)  # Device Reset
        allDataPackage.append(actSettings.trControl)  # Transfer Control
        allDataPackage.append(actSettings.numOfDevices)  # Number of Devices
        j = 0
        while j < 16:
            k = 0
            # Write Address here !!!!
            while k < 12:
                allDataPackage.append(nodeDataBase[j].txAddress[k])
                k += 1
            k = 12
            while k < 18:
                allDataPackage.append(actSettings.numOfDevices)
                k += 1
            j += 1
        CRCforAllDataPackage = 0
        for x in allDataPackage:
            CRCforAllDataPackage = CRCforAllDataPackage ^ x
        allDataPackage.append(CRCforAllDataPackage)
        # print('to send message size'+str(len(allDataPackage)))

    def sendReadAllPackage(self):
        global transportDataGroup
        global recSettings
        # cls()
        # print('Read All Bytes')
        # prepare package
        SPIReceiver.serializeDataBaseToMessage(self)
        transportDataGroup = spi.xfer2(allDataPackage)
        SPIReceiver.readAndSaveContentOfResult(self)

    def readAndSaveContentOfResult(self):
        global transportDataGroup
        global recSettings
        global newDataFlag
        global nodeDataBase
        # print('Size of received message' + str(len(bytes(transportDataGroup))))
        # uncomment for spi package#print(bytes(transportDataGroup) + "\n")
        logging.info("Read and Save Content from SPI")
        logging.info("Content: " + ''.join(format(x, '02x') for x in transportDataGroup))
        # if transportDataGroup[0] == 0xAA:
        #    print("Data message arrives from Gecko")
        recSettings.radioMode = transportDataGroup[1]
        recSettings.deviceReset = transportDataGroup[2]
        recSettings.trControl = transportDataGroup[3]
        recSettings.numOfDevices = transportDataGroup[4]
        # print('Num of devices :' + str(recSettings.numOfDevices) + "\n")
        logging.info("Num of devices: " + str(recSettings.numOfDevices))
        # node 0
        nodeNum = 0
        while nodeNum < 16:
            nodeDataBase[nodeNum].nodeNumber = nodeNum
            nodeDataBase[nodeNum].txAddress = transportDataGroup[(nodeNum * 18) + 5:(nodeNum * 18) + 17]
            nodeDataBase[nodeNum].vBattery = transportDataGroup[(nodeNum * 18) + 17:(nodeNum * 18) + 19]
            nodeDataBase[nodeNum].stBattery = transportDataGroup[(nodeNum * 18) + 19]
            nodeDataBase[nodeNum].tempDevice = transportDataGroup[(nodeNum * 18) + 20]
            nodeDataBase[nodeNum].mForce = transportDataGroup[(nodeNum * 18) + 21:(nodeNum * 18) + 23]
            # pprint(vars(nodeDataBase[nodeNum]))
            nodeNum += 1
            # txAddress  # 12 bytes [0h-0Bh]
            # vBattery   # 2 bytes  [0Ch-0Dh]
            # stBattery  # 1 byte   [0Eh]
            # tempDevice # 1 byte   [0Fh]
            # mForce     # 2 bytes  [10h-11h]

        newDataFlag = True


class SocketServer:
    global sock
    """ Simple socket server that listens to one single client. """

    def __init__(self, host='0.0.0.0', port=2010):
        global sock
        """ Initialize the server with a host and port to listen to. """
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.host = host
        self.port = port
        sock.bind((host, port))
        sock.listen(1)
        self._running = True

    def terminate(self):
        self._running = False

    def run(self):
        global haveSocketOpened
        while self._running:
            self.run_server()
            haveSocketOpened = False
            time.sleep(5)
            self.close()
            time.sleep(5)
            self.__init__()
        print('Exiting')

    def close(self):
        global sock
        """ Close the server socket. """
        print('Closing server socket (host {}, port {})'.format(self.host, self.port))
        if sock:
            sock.close()
            sock = None

    def sendSocketMessage(self, message):
        global sock
        global haveSocketOpened
        global client_sock
        # print('Send Socket message : ')
        try:
            if haveSocketOpened:
                client_sock.send(message)
        except:
            logging.error("Socket server - error while sending messages")
            print("Error sending Socket message")

    def run_server(self):
        global sock
        global client_sock
        global haveSocketOpened
        """ Accept and handle an incoming connection. """
        print('Starting socket server (host {}, port {})'.format(self.host, self.port))
        client_sock, client_addr = sock.accept()
        print('Client {} connected'.format(client_addr))
        haveSocketOpened = True
        stop = False
        while not stop:
            if client_sock:
                # Check if the client is still connected and if data is available:
                try:
                    rdy_read, rdy_write, sock_err = select.select([client_sock, ], [], [])
                except select.error:
                    print('Select() failed on socket with {}'.format(client_addr))
                    logging.error("Socket server - Select() failed on socket with {}")
                    return 1
                if len(rdy_read) > 0:
                    try:
                        read_data = client_sock.recv(255)
                        # Check if socket has been closed
                        if len(read_data) == 0:
                            print('{} closed the socket.'.format(client_addr))
                            stop = True
                            haveSocketOpened = False
                        else:
                            print('>>> Received: {}'.format(read_data.rstrip()))
                            if read_data.rstrip() == 'quit':
                                stop = True
                                haveSocketOpened = False
                            else:
                                client_sock.send(read_data)

                    except ConnectionResetError:
                        print('Client has disconnected while reading from the socket, go back to listening')
                        stop = True
                        haveSocketOpened = False
            else:
                print("No client is connected, SocketServer can't receive data")
                stop = True
                haveSocketOpened = False
        # Close socket
        print('Closing connection with {}'.format(client_addr))
        client_sock.close()
        haveSocketOpened = True
        return 0


class MesurementNode:
    def __init__(self, nodeNumberNode):
        # Number of the actual node
        self.nodeNumber = nodeNumberNode
        # ZigBee Address of the actual node
        self.txAddress = []
        i = 0
        while i < 12:
            self.txAddress.append(nodeNumberNode)
            i += 1
        # Battery voltage of the actual node
        self.vBattery = []
        i = 0
        while i < 2:
            self.txAddress.append(nodeNumberNode)
            i += 1
        # Battery charging state of the actual node
        self.stBattery = nodeNumberNode
        # Temperature value of the actual node
        self.tempDevice = nodeNumberNode
        # Measured force of the actual node
        self.mForce = []
        i = 0
        while i < 2:
            self.txAddress.append(nodeNumberNode)
            i += 1

        # txAddress  # 12 bytes [0h-0Bh]
        # vBattery   # 2 bytes  [0Ch-0Dh]
        # stBattery  # 1 byte   [0Eh]
        # tempDevice # 1 byte   [0Fh]
        # mForce     # 2 bytes  [10h-11h]


class RfIDSettings:
    def __init__(self):
        # RfID reader ID
        self.readerID = []  # 4 bytes
        byteNum = 0
        while byteNum < 4:
            self.readerID.append(0x01)
            byteNum += 1
        # Battery of RfID reader voltage
        self.batteryVoltage = []  # 2 bytes
        byteNum = 0
        while byteNum < 2:
            self.batteryVoltage.append(0x00)
            byteNum += 1
        # Temperature of the RfID reader
        self.temperature = -127  # 1 byte
        # Stored positions
        self.storedPositions = []  # 16 Bytes
        nodeNum = 0
        while nodeNum < 16:
            self.storedPositions.append(0xff)
            nodeNum += 1

        # Stored addresseses
        self.storedAddresses = []  # 16 ZigBee Addresses
        nodeNum = 0
        while nodeNum < 16:
            self.storedAddresses.append(ZigBeeAddress())
            nodeNum += 1


class ZigBeeAddress:
    def __init__(self):
        # Stored addresses
        self.ownAddress = []  # 12 Bytes
        byteNum = 0
        while byteNum < 12:
            self.ownAddress.append(0xff)
            byteNum += 1


class CommandSettings:
    def __init__(self):
        # Radio Mode
        self.radioMode = 2  # 0 - Radio Not Active, 1 - Listen Only, 2 - Radio Active
        # Device Reset
        self.deviceReset = 0xAA  # 0xAA ??
        # Transport / Measure control
        self.trControl = 0  # 0 - Measure Stop, 1 - Measure Start, 2 - Measure Pause
        # Number of devices
        self.numOfDevices = 0


def initDataBase():
    nodeDataBase.append(MesurementNode(0))
    nodeDataBase.append(MesurementNode(1))
    nodeDataBase.append(MesurementNode(2))
    nodeDataBase.append(MesurementNode(3))
    nodeDataBase.append(MesurementNode(4))
    nodeDataBase.append(MesurementNode(5))
    nodeDataBase.append(MesurementNode(6))
    nodeDataBase.append(MesurementNode(7))
    nodeDataBase.append(MesurementNode(8))
    nodeDataBase.append(MesurementNode(9))
    nodeDataBase.append(MesurementNode(9))
    nodeDataBase.append(MesurementNode(10))
    nodeDataBase.append(MesurementNode(11))
    nodeDataBase.append(MesurementNode(12))
    nodeDataBase.append(MesurementNode(13))
    nodeDataBase.append(MesurementNode(14))
    nodeDataBase.append(MesurementNode(15))


def initTestPackage():
    testPackage.append(0xaa)
    testPackage.append(1)
    testPackage.append(2)
    testPackage.append(4)
    testPackage.append(8)
    testPackage.append(16)
    testPackage.append(32)
    testPackage.append(64)
    testPackage.append(128)
    testPackage.append(1)
    testPackage.append(2)
    testPackage.append(4)
    testPackage.append(8)
    testPackage.append(16)
    testPackage.append(32)
    testPackage.append(64)
    testPackage.append(128)
    testPackage.append(1)
    testPackage.append(2)
    CRCforTestPackage = 0
    for x in testPackage:
        CRCforTestPackage = CRCforTestPackage ^ x
    testPackage.append(CRCforTestPackage)


def cls():
    u = 0
    while u < 7:
        print('\n')
        u += 1


# def uploadDataBase(self):
# read out all data from HW for all nodes in the system

# Init database
initDataBase()

# Init Test package
initTestPackage()

actSettings = CommandSettings()
recSettings = CommandSettings()

# Create Class
BTserver = BluetoothServer()
# Create Thread
BTserverThread = Thread(target=BTserver.run)
# Start Thread
BTserverThread.start()

# Create Class
GPSConnection = GPSUARTConnection()
# Create Thread
GPSConnectionThread = Thread(target=GPSConnection.run)
# Start Thread
GPSConnectionThread.start()

# Create Class
SocketServerClass = SocketServer()
# Create Thread
SocketServerThread = Thread(target=SocketServerClass.run)
# Start Thread
SocketServerThread.start()

# Create Class
SPIRec = SPIReceiver()
# Create Thread
SPIRecThread = Thread(target=SPIRec.run)
# Start Thread
SPIRecThread.start()



send1Message = True
send2Message = False

Exit = False  # Exit flag
print('Waiting for data')
while Exit == False:
    if IO.input(5) == IO.LOW:
        print("1 Button was pushed!")
        # zummer.on()
        send1Message = True
        send2Message = False
    if IO.input(6) == IO.LOW:
        print("2 Button was pushed!")
        # zummer.on()
        send1Message = False
        send2Message = True
    if IO.input(12) == IO.LOW:
        print("3 Button was pushed!")
        # zummer.on()
        send1Message = False
        send2Message = False
    if IO.input(5) == IO.HIGH & IO.input(6) == IO.HIGH & IO.input(12) == IO.HIGH:
        zummer.off()

    SPIRec.sendReadAllPackage()

    time.sleep(3)
print('End of resident application')