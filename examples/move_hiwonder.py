"""
sudo apt-get install libhidapi-hidraw0 libhidapi-libusb0
pip3 install --user hid

Notes:
- servos_off will power off the servos
- sending a movement command wakes up unpowered servos
- position readouts are sadly pretty slow, they can take up to 450ms
"""

from time import sleep
import easyhid
import numpy as np

def itos(v):
    lsb = v & 0xFF
    msb = v >> 8
    return lsb, msb

class XArm():
    def __init__(self, pid=22352):

        # Stores an enumeration of all the connected USB HID devices
        en = easyhid.Enumeration()
        # return a list of devices based on the search parameters
        devices = en.find()
        # Find the HiWonder device by its product string
        for dev in devices:
            if 'hiwonder' in dev.product_string.lower():
                self.dev = dev
                break
        # print a description of the devices found
        #for dev in devices:
        #    print(dev.description())

        assert len(devices) > 0

        # open a device
        self.dev.open()
        print('Connected to xArm device')

    def __del__(self):
        print('Closing xArm device')
        self.dev.close()

    def move_to(self, id, pos, time=0):
        """
        CMD_SERVO_MOVE
        0x55 0x55 len 0x03 count [time_lsb time_msb, id, pos_lsb pos_msb]
        Servo position is in range [0, 1000]
        """
        t_lsb, t_msb = itos(time)
        p_lsb, p_msb = itos(pos)
        self.dev.write([0x55, 0x55, 8, 0x03, 1, t_lsb, t_msb, id, p_lsb, p_msb])

    def move_all(self, poss, time=500):
        """
        Set the position of all servos at once
        """
        
        for i in range(6):
            self.move_to(id=i+1, pos=poss[i], time=time)
            sleep(2)

    def servos_off(self):
        self.dev.write([0x55, 0x55, 9, 20, 6, 1, 2, 3, 4, 5, 6])

    def read_pos(self):
        """
        Read the position of all 6 servos
        ServoPositionRead 21 (byte)count { (byte)id }; (byte)count { (byte)id (ushort)position }
        """

        self.dev.write([
            0x55, 0x55,
            9,  # Len
            21, # Cmd
            6,  # Count
            1,
            2,
            3,
            4,
            5,
            6
        ])

        ret = self.dev.read()
        count = ret[4]
        assert count == 6

        poss = []
        for i in range(6):
            id = ret[5 + 3*i]
            p_lsb = ret[5 + 3*i + 1]
            p_msb = ret[5 + 3*i + 2]
            pos = (p_msb << 8) + p_lsb
            poss.append(pos)

        return np.array(poss)

    def rest(self):
        self.move_all([500, 500, 200, 900, 800, 500], time=1500)
        time.sleep(2)
        self.servos_off()



def demo():
    arm = XArm()
    pos = 490
    # To the right
    arm.move_all([pos]*6)
demo()
