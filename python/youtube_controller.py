from bdb import Breakpoint
from ctypes import cast, POINTER
from comtypes import CLSCTX_ALL
from pycaw.pycaw import AudioUtilities, IAudioEndpointVolume
import pyautogui
import serial
import argparse
import time
import logging

# How our protocol works
#   B     A     X
#   ^     ^     ^ 
# Head   Data  EOP

class MyControllerMap:
    def __init__(self):
        self.button = {
            "A": ["T", "KEYDOWN T"],
            "a": ["T", "KEYUP T"],
            "B": ["Y", "KEYDOWN Y"],
            "b": ["Y", "KEYUP Y"],
            "C": ["U", "KEYDOWN U"],
            "c": ["U", "KEYUP U"],
            "D": ["I", "KEYDOWN I"],
            "d": ["I", "KEYUP I"], 
            "E": ["O", "KEYDOWN O"],
            "e": ["O", "KEYUP O"],  
            "F": ["P", "KEYDOWN F"],
            "f": ["P", "KEYUP F"],
            "G": ["´", "KEYDOWN ´"],
            "g": ["´", "KEYUP ´"],
            "I": ["6", "KEYDOWN 6"],
            "i": ["6", "KEYUP 6"],
            "J": ["7", "KEYDOWN 7"],
            "j": ["7", "KEYUP 7"],
            "K": ["9", "KEYDOWN 9"],
            "k": ["9", "KEYUP 9"],
            "L": ["0", "KEYDOWN 0"],
            "l": ["0", "KEYUP 0"],
            "M": ["-", "KEYDOWN -"],
            "m": ["-", "KEYUP -"],
            } 

        self.volume = {
            15: -60,
            14: -56,
            13: -52,
            12: -48,
            11: -44,
            10: -40,
            9: -38,
            8: -36,
            7: -32,
            6: -28,
            5: -24,
            4: -20,
            3: -16,
            2: -8,
            1: -4,
            0: 0
        }

class SerialControllerInterface:
    # Protocolo
    # byte 1 -> Botão 1 (estado - Apertado 1 ou não 0)
    # byte 2 -> EOP - End of Packet -> valor reservado 'X'

    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate=baudrate)
        self.mapping = MyControllerMap()
        self.incoming = '0'
        pyautogui.PAUSE = 0  ## remove delay
        
        # Get default audio device using PyCAW
        global devices, interface, volume
        devices = AudioUtilities.GetSpeakers()
        interface = devices.Activate(IAudioEndpointVolume._iid_, CLSCTX_ALL, None)
        volume = cast(interface, POINTER(IAudioEndpointVolume))

        global old_data
        old_data = 0

    def update(self):

        # Sync protocol
        while self.incoming != b'X':
            self.incoming = self.ser.read()
            logging.debug("Received INCOMING: {}".format(self.incoming))

        # Receiving the data 
        head_bin =  self.ser.read()
        data_bin =  self.ser.read()

        # Reading the EOP
        self.incoming = self.ser.read()
        
        head = head_bin.decode('utf-8')

        if head == 'H':
            data = data_bin.decode('utf-8') 
            logging.debug("Handshake Received")
            
            self.ser.write(b'h')

        if head == 'V':

            data = int.from_bytes(data_bin, byteorder ='big')
            logging.debug("Received DATA - VOLUME: {}".format(data))

            # Get current volume 
            volume.SetMasterVolumeLevel(self.mapping.volume[data], None)

            
        if head == 'B':

            data = data_bin.decode('utf-8')
            logging.debug("Received DATA - BUTTON: {}".format(data))

            # Sending the instruction
            if data.isupper():
                pyautogui.keyDown(self.mapping.button[data][0])
            else:
                pyautogui.keyUp(self.mapping.button[data][0])


class DummyControllerInterface:
    def __init__(self):
        self.mapping = MyControllerMap()

    def update(self):
        pyautogui.keyDown(self.mapping.button['A'])
        time.sleep(0.1)
        pyautogui.keyUp(self.mapping.button['A'])
        logging.info("[Dummy] Pressed A button")
        time.sleep(1)


if __name__ == '__main__':
    interfaces = ['dummy', 'serial']
    argparse = argparse.ArgumentParser()
    argparse.add_argument('serial_port', type=str)
    argparse.add_argument('-b', '--baudrate', type=int, default=9600)
    argparse.add_argument('-c', '--controller_interface', type=str, default='serial', choices=interfaces)
    argparse.add_argument('-d', '--debug', default=False, action='store_true')
    args = argparse.parse_args()
    if args.debug:
        logging.basicConfig(level=logging.DEBUG)

    print("Connection to {} using {} interface ({})".format(args.serial_port, args.controller_interface, args.baudrate))
    if args.controller_interface == 'dummy':
        controller = DummyControllerInterface()
    else:
        controller = SerialControllerInterface(port=args.serial_port, baudrate=args.baudrate)

    while True:
        controller.update()