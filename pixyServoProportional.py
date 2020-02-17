#Scroll down to line 230 to see the proportional control algorithm in action

import board
import time
import pulseio
import busio

import struct

i2c = busio.I2C(board.SCL, board.SDA)
from adafruit_bus_device.i2c_device import I2CDevice

from adafruit_motor import servo

#A frequency of 50 Hz means that the period is 1/50 which is 2 milliseconds. This is the standard width of a servo pulse.
pwm = pulseio.PWMOut(board.A2, frequency=50)

servoOne = servo.Servo(pwm, min_pulse=750, max_pulse=2250)

#Start the initial angle of the servo at 0
servoOne.angle = 0
#servo is initially moving in the increasing direction
direction = 1



class Pixy2():


    def __init__(self, i2c, address):
        self.address = address
        self.i2c_device = I2CDevice(i2c, address)
        self.blocks = []


    def set_lamp(self, on):
        '''
        Turn on or off the Pixy2's lamp.

        :param on: True or False on whether the Pixy2's lamps is on or off.
        :return: Nothing.
        '''

        with self.i2c:
            out = [174, 193, 22, 2, 1 if on else 0, 0]
            for msg in out:
                self.i2c_device.write(bytes(out), stop=False)


    def set_led(self, red, green, blue):
        """
        Set the Pixy2's RGB LED.

        :param red: 0-255.
        :param green: 0-255.
        :param blue: 0-255.
        :return: Nothing
        """
        out = [174, 193, 20, 3, red, green, blue]

        with self.i2c_device:
            self.i2c_device.write(bytes(out), stop=False)


    def get_resolution(self):
        """
        Return the width and height of the camera.

        :return: width, height (0-511). None if the checksum didn't match.
        """
        out = [
            # 2 sync bytes, type packet, length payload, unused type
            174, 193, 12, 1, 0
        ]
        with self.i2c_device:
            self.i2c_device.write(bytes(out), stop = False)
            inp = bytearray(10)
            self.i2c_device.readinto(inp)
            checksum = struct.unpack('<H', bytes(inp[4:6]))[0]
            if checksum == sum(inp[6:10]):
                width, height = struct.unpack('<HH', bytes(inp[6:10]))
                return width, height
            else:
                return None

    def get_version(self):
            """
            Get the hardware and software version of the Pixy2.

            :return: hw, sw
            """
            out = [
                # 2 sync bytes, type packet, length payload
                174, 193, 14, 0
            ]
            print('get version from pixy2')
            with self.i2c_device:
                self.i2c_device.write(bytes(out), stop = False)
                inp = bytearray(14)
                self.i2c_device.readinto(inp)

                #hw = unpack_bytes(inp[6:8], big_endian=False)
                hw = struct.unpack('H', bytes(inp[6:8]))[0]
                major = inp[8]
                minor = inp[9]
                #build = unpack_bytes(inp[10:12], big_endian=False)
                build = struct.unpack('H', bytes(inp[10:12]))[0]
                fw_type = inp[12]
                fw = '{}.{}.{}-{}'.format(major, minor, build, chr(fw_type))
                return hw, fw
    def get_fps(self):
            """
            Get the Pixy2's camera FPS.

            :return: The FPS as an integer.
            """
            out = [
                # 2 sync bytes, type packet, length payload
                174, 193, 24, 0
            ]
            #print('get fps from pixy2')
            with self.i2c_device:

                self.i2c_device.write(bytes(out), stop = False)
                inp = bytearray(10)
                self.i2c_device.readinto(inp)

                fps = struct.unpack('<I', bytes(inp[6:10]))[0]

            return fps

    def get_blocks(self, sigmap, maxblocks):
        """
        Get detected blocks from the Pixy2.

        :param sigmap: Indicates which signatures to receive data from.
        0 for none, 255 for all, all the rest it's in between.
        :param maxblocks: Maximum blocks to return.
        0 for none, 255 for all of them, all the rest it's in between.
        :return: signature, X center of block (px) (0-315), Y center of block (px) (0-207), width
        of block (px) (0-316), height of block (px) (0-208), angle of color-code in degrees (-180 - 180)
        w/ 0 if not a color code, tracking index (0-255), age or the number of frames this
        block has been tracked for (0-255) - it stops incrementing at 255. Returned as a list of pairs.
        :return: None if it hasn't detected any blocks or if the process has encountered errors.
        """
        out = [ 174, 193, 32, 2, sigmap, maxblocks]
         # 2 sync bytes, type packet, length payload,
         # sigmap, max blocks to return
        #print('detect pixy2 blocks')
        with self.i2c_device:
            self.i2c_device.write(bytes(out), stop=False)
            result = bytearray(20)

            self.i2c_device.readinto(result)

            #for msg in result:
            #    print(int(msg))
            type_packet = result[2]

            if type_packet == 33:
                payload_length = result[3]

                inp = result[4:]


                checksum = struct.unpack('<H', bytes(inp[0:2]))[0]

                if(checksum == sum(inp[2:])):
                    block_length = 14
                    num_blocks = payload_length // block_length
                    blocks = []

                    if(num_blocks > 0):
                        for i in range(num_blocks):
                            data = struct.unpack('<5HhBB', bytes(inp[(i*block_length+2):(i+1)*block_length+2]))
                            blocks.append(data)
                        #print('pixy2 detected {} blocks'.format(no_blocks))
                        self.blocks = blocks[0]

                    else:
                        self.blocks = []
                    return self.blocks


            else:
                #print('checksum doesn\'t match for the detected blocks')
                self.blocks = []
                return None






        #print('pixy2 is busy or got into an error while reading blocks')
        #self.blocks = []
        return None
    def isTracking(self):
        if(len(self.blocks)>0):
            return True
        return False

class TrackingData():
    def __init__(self, block):
        if(block != None):
            self.x = block[1]/315.0 #number from 0.0 to 1.0
            self.y = block[2]/207.0 #number from 0.0 to 1.0
            self.width = block[3] #pixels
            self.height = block[4] #pixels

        else:
            self.x = None #number from 0.0 to 1.0
            self.y = None #number from 0.0 to 1.0
            self.width = None #pixels
            self.height = None #pixels

myPixy = Pixy2(i2c, 0x54)

myPixy.set_led(255, 0,0)
time.sleep(0.1)
myPixy.set_led(0, 255,0)
time.sleep(0.1)
myPixy.set_led(0, 0,255)
time.sleep(0.1)
myPixy.set_led(0, 0,0)
time.sleep(0.1)
servoOne.angle = 10


#Beginning of Proportional control algorithm

#Define a constant for proportional control
kP = 6

#Define a setpoint for the center x coordinate of the camera (remember that the camera gives a number from 0 to 1.)
setpoint = 0.5

while True:
    try:
        #Get data from the camera
        tracked_object = TrackingData(myPixy.get_blocks(255,1))

        #if there is an object in the camera view:
        if(tracked_object.x != None):

            #print("x: {} y: {} height: {} width: {} servo: {}".format(tracked_object.x, tracked_object.y, tracked_object.width, tracked_object.height, servoOne.angle))

            #calculate the error as the difference between the setpoint and the current position of the object
            error =  setpoint - tracked_object.x

            #calculate the output as proportional to the error
            output = kP*error

            #calculate the new angle of the servo as the old angle plus the output.
            newAngle = servoOne.angle + output

            #Prevent out-of-range errors by making sure the angle does not go over 180 or below 0
            if (newAngle > 180):
                newAngle = 180
            elif(newAngle < 0):
                newAngle = 0

            #set the servo angle to this new angle
            servoOne.angle = newAngle


        #else:
            #print("No color is being tracked.")
        time.sleep(0.01)
    except:
        print("error has occurred, trying again")
        time.sleep(0.5)

