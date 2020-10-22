import Tkinter as tk
import serial
import time
import inputs
# Main thread
from threading import Thread

global uartArduinoOK
global ser
global arDuinoSerialPort
global MVSThread
global GPPThread
global rotate_left
global rotate_right
global pads
global currentCent
global currentSection
global leftValue
global rightValue

rotate_left = False
rotate_right = False
pads = inputs.devices.gamepads
currentCent = 0
currentSection = 0
leftValue = 1300
rightValue = 1300


uartArduinoOK = False

class Display(tk.Frame):
    global arDuinoSerialPort

    def __init__(self, *args, **kwargs):
        tk.Frame.__init__(self, *args, **kwargs)

        self.configure(background='bisque')
        self.w = 700
        self.h = 100
        self.sw = self.winfo_screenwidth()
        self.myParent = self.winfo_toplevel()
        self.sh = self.winfo_screenheight()
        self.x = (self.sw - self.w) / 2
        self.y = (self.sh - self.h) / 2

        self.b1 = tk.Button(self, text="Stop", command=self.c0, padx=5, bg='green')
        self.b1.pack(side="left")
        self.b1 = tk.Button(self, text="Fast Test", command=self.c1, padx=5, bg='red')
        self.b1.pack(side="left")
        self.b2 = tk.Button(self, text="Middle test", command=self.c2, padx=5, bg='yellow')
        self.b2.pack(side="left")
        self.b3 = tk.Button(self, text="Slow test", command=self.c3, padx=5, bg='white')
        self.b3.pack(side="left")
        self.b4 = tk.Button(self, text="Controller test", padx=5, bg='orange')
        self.b4.pack(side="left")

        self.b4.bind("<ButtonPress>", self.c4_onPress);

        self.b5 = tk.Button(self, text="Rotate Left", padx=5, bg='orange')
        self.b5.pack(side="left")

        self.b5.bind("<ButtonPress>", self.c5_onPress);
        self.b5.bind("<ButtonRelease>", self.c0_onPress);

        self.b6 = tk.Button(self, text="Go Forward", padx=5, bg='orange')
        self.b6.pack(side="left")

        self.b6.bind("<ButtonPress>", self.c6_onPress);
        self.b6.bind("<ButtonRelease>", self.c0_onPress);

        self.b7 = tk.Button(self, text="Go Back", padx=5, bg='orange')
        self.b7.pack(side="left")

        self.b7.bind("<ButtonPress>", self.c7_onPress);
        self.b7.bind("<ButtonRelease>", self.c0_onPress);

        self.be = tk.Button(self, text="Exit", command=self.myParent.destroy)
        self.be.pack(side="bottom")

        self.w = tk.Scale(self, from_=0, to=99, orient=tk.HORIZONTAL, command=self.print_value)
        self.w.set(50)
        self.w.pack(side="left")


    def c0(self):
        global arDuinoSerialPort
        arDuinoSerialPort.send("0")
        print("0")

    def c0_onPress(self, event):
        global arDuinoSerialPort
        arDuinoSerialPort.send("0")
        print("Stop moving robot!")

    def c1(self):
        global arDuinoSerialPort
        arDuinoSerialPort.send("1")
        print("1")

    def c2(self):
        global arDuinoSerialPort
        arDuinoSerialPort.send("2")

    def c3(self):
        global arDuinoSerialPort
        arDuinoSerialPort.send("3")
        print("3")

    def c4_onPress(self, event):
        MVSThread.start()
        GPPThread.start()

    def c4_onRelease(self, event):
        MVS.stop()

    def c5_onPress(self, event):
        global arDuinoSerialPort
        arDuinoSerialPort.send("8 31 29 29")
        print("Rotate the robot Left")

    def c6_onPress(self, event):
        global arDuinoSerialPort
        arDuinoSerialPort.send("8 56 29 50")
        print("Move the robot Forward")

    def c7_onPress(self, event):
        global arDuinoSerialPort
        arDuinoSerialPort.send("8 31 56 50")
        print("Move the robot Backward")

    def print_value(self, val):
        print
        val


class SerialPort:
    def __init__(self):
        global ser
        global uartArduinoOK
        self.initUART()

    def initUART(self):
        global uartArduinoOK
        global ser
        # print('OPENING serial port 0')
        try:
            ser = serial.Serial(
                port='/dev/ttyACM0',
                baudrate=115200,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
            )
            print('Arduino connected succesfully')
        except:
            print("Error opening serial port for Arduino")


    def close(self):
        global ser
        ''' Close the serial port.'''
        ser.close()

    def send(self, msg):
        global ser
        ser.write(msg)
        ser.flush
        print("Send message to arDuino:" + msg)

    def recv(self):
        global ser
        return ser.readline()


class MotorValuesSetter:
    _running = True
    def __init__(self):
        global arDuinoSerialPort

    def run(self):
        global arDuinoSerialPort
        global leftValue
        global rightValue

        print('MotorValuesSetter has been started!!')

        while self._running:
            time.sleep(0.45)
            self.sendValues()

    def stop(self):
        self._running = False

    def sendValues(self):
        global leftValue
        global rightValue
        global arDuinoSerialPort
        arDuinoSerialPort.send("8 " + str(1300) + " " + str(rightValue) + " " + str(leftValue))

class InitDelayClass:
    def run(self):
        global MVSThread
        global GPPThread

        time.sleep(10)

        MVSThread.start()
        GPPThread.start()


class GamePadPoller:
    def __init__(self):
        self._running = True

    def terminate(self):
        self._running = False

    def sendControlSignal(self,rpm,direction,motor): # 0 - forward, 1 - backward
        global currentCent
        global arDuinoSerialPort
        centWhich = rpm - (rpm % 100)
        if centWhich != currentCent:
            currentCent = centWhich
            if motor == "L":  # this for the motor Left
                controlType = "9 "
            else:
                if motor == "R":
                    controlType = "A "
            if direction == 0:
                arDuinoSerialPort.send(controlType + str(currentCent + 450))
                print("Rise speed to rpm (" + motor + "): " + str(currentCent + 450))
            else:
                arDuinoSerialPort.send(controlType + str((-1) * (currentCent + 450)))
                print("Rise speed to rpm (" + motor + "): " + str((-1) * (currentCent + 450)))

    def sendPPMControlSignal(self,measurement,motor):
        global currentSection
        global arDuinoSerialPort
        #print("Controller value: " + str(measurement))
        sectionWhich = measurement - (measurement % 200)
        if sectionWhich != currentSection:
            currentSection = sectionWhich
            if motor == "L":  # this for the motor Left
                controlType = "7 "
            else:
                if motor == "R":
                    controlType = "6 "
            arDuinoSerialPort.send(controlType + str(currentSection))
            print("Rise speed to ppm (" + motor + "): " + str(currentSection))



    def run(self):
        global arDuinoSerialPort
        global rotate_left
        global rotate_right
        global pads
        global leftValue
        global rightValue

        print('GamePad controller has been started!!')

        while self._running:
            events = inputs.get_gamepad()
            for event in events:
                if event.code == 'BTN_TL' and event.state == 1 and rotate_left == False:
                    print('Rotate Left! (Now stop the motor)')
                    arDuinoSerialPort.send("0")
                    rotate_left = True
                if event.code == 'BTN_TL' and event.state == 0:
                    print('Stop Rotate Left! (Now stop the motor)')
                    arDuinoSerialPort.send("0")
                    rotate_left = False
                if event.code == 'BTN_TR' and event.state == 1 and rotate_left == False:
                    print('Rotate Right! (Now stop the motor)')
                    arDuinoSerialPort.send("0")
                    rotate_right = True
                if event.code == 'BTN_TR' and event.state == 0:
                    print('Stop Rotate Right! (Now stop the motor)')
                    arDuinoSerialPort.send("0")
                    rotate_right = False
                if event.code == 'ABS_Y':
                    # print('Left Forward, speed:' + str(event.state))
                    if event.state < 127:
                        # rpm controlled version
                            #print('Left Forward, speed:' + str(127 - event.state))          # 127 is the default value at this channel
                            #self.sendControlSignal(((127 - event.state) * 5), 0, "L")
                        # ppm controlled version
                        #self.sendPPMControlSignal(1300 + ((127 - event.state)  * 3), "L")
                        leftValue = 1300 + ((127 - event.state)  * 2)

                    else:
                        # rpm controlled version
                            #print('Left Backward, speed:' + str(event.state - 127))         # 127 is the default value at this channel
                            #self.sendControlSignal((event.state - 127) * 3, 1, "L")
                        # ppm controlled version
                        #self.sendPPMControlSignal(1300 - ((event.state - 127) * 3), "L")
                        leftValue = 1300 - ((event.state - 127) * 2)
                if event.code == 'ABS_RZ':
                    # print('Right Forward, speed:' + str(event.state))
                    if event.state < 127:
                        # rpm controlled version
                            #print('Right Forward, speed:' + str(127 - event.state))         # 127 is the default value at this channel
                            #self.sendControlSignal(((127 - event.state) * 5), 0, "R")
                        # ppm controlled version
                        #self.sendPPMControlSignal(1300 + ((127 - event.state) * 3), "R")
                        rightValue = 1300 - ((127 - event.state) * 2)
                    else:
                        # rpm controlled version
                            #print('Right Backward, speed:' + str(event.state - 127))        # 127 is the default value at this channel
                            #self.sendControlSignal((event.state - 127) * 3, 1, "R")
                        # ppm controlled version
                        #self.sendPPMControlSignal(1300 - ((event.state - 127) * 3), "R")
                        rightValue = 1300 + ((event.state - 127) * 2)

                print("EventCode:" + str(event.code))


# Create Class for connection to ArDuino
arDuinoSerialPort = SerialPort()

# Create Class for polling Game Pad controller
GPP = GamePadPoller()
# Create Thread
GPPThread = Thread(target=GPP.run)

MVS = MotorValuesSetter()
MVSThread = Thread(target=MVS.run)

InitDC = InitDelayClass()
InitDCThread = Thread(target=InitDC.run())
InitDCThread.start()

if len(pads) == 0:
    raise Exception("Couldn't find any Gamepads!")

if __name__ == "__main__":
    root = tk.Tk()
    Display(root).pack(side="top", fill="both", expand=True)
    root.title("Dummy Control")
    root.geometry("800x100")
    print('Mainloop started ...')
    root.mainloop()

