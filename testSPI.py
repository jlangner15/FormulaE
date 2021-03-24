#import spidev
import time

class Accelerometer:
    gravity = 9.80665
    scalefactor = 0

    def __init__(self, chip, acc):
        self.spi = None
        """
        All data register maps and spec information regarding mpu9250 chip can be found at these two links
        https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
        https://invensense.tdk.com/wp-content/uploads/2015/02/RM-MPU-9250A-00-v1.6.pdf
        """

        try:
            self.spi = spidev.SpiDev()
            self.spi.open(0, 0)
            self.spi.max_speed_hz = 1000000
            self.spi.mode = 3

        except Exception as error:
            print(error)

            if self.spi:
                self.spi.close()
                self.spi = None

        if chip == '9250':
            self.accel_x_high = 0x3B
            self.accel_x_low = 0x3C

            self.accel_y_high = 0x3D
            self.accel_y_low = 0x3E

            self.accel_z_high = 0x3F
            self.accel_z_low = 0x40

        time.sleep(0.1)

        self.acc = acc

    def accelerometer_range_scalefactor(self):

        # Setting the scalefactor for each Force Sensitivity Level

        if self.acc == 2:
            self.scalefactor = 16384.0
            self.range = 0x00

        elif self.acc == 4:
            self.scalefactor = 8192.0
            self.range = 0x08

        elif self.acc == 8:
            self.scalefactor = 4096.0
            self.range = 0x10

        elif self.acc == 16:
            self.scalefactor = 2048.0
            self.range = 0x18

        else:
            print("Invalid range input")


    def get_bit(self, direction):
        """
        Each instance of different directions must create a bytearray list to perform a transaction
        in the following form [addres high, address low, garbage]
        Each address must add 128 or 0x80 to specify read function for the chip as mentioned in the spec
        After the transfer we must shift back 8 to properly add the two
        """

        if direction == 'x':
            xL = bytearray([self.accel_x_high + 128, self.accel_x_low + 128, 129])
            x_read = []
            x_read = self.spi.xfer(xL)
            conv = (x_read[0] << 8) + x_read[1]
            return conv

        elif direction == 'y':
            yL = bytearray([self.accel_y_high + 128, self.accel_y_low + 128, 129])
            y_read = []
            y_read = self.spi.xfer(yL)
            conv = (y_read[0] << 8) + y_read[1]
            return conv

        elif direction == 'z':
            zL = bytearray([self.accel_z_high + 128, self.accel_z_low + 128, 129])
            z_read = []
            z_read = self.spi.xfer(zL)
            conv = (z_read[0] << 8) + z_read[1]
            return conv

        else:
            print("Enter valid direction")

    def getAccelerometerData(self):
        self.accelerometer_range_scalefactor()

        # Here the data for each acceleration direction (x,y,z) is being read and converted to signed 16 bit value

        ax = self.get_bit('x')
        ay = self.get_bit('y')
        az = self.get_bit('z')

        # Signed 16 bit value is converted into G force

        x = round((ax / self.scalefactor) * self.gravity, 3)
        y = round((ay / self.scalefactor) * self.gravity, 3)
        z = round((az / self.scalefactor) * self.gravity, 3)

        return x, y, z

    def Accelerometer_Data(self, t):
        startTime = time.time()
        while (time.time() < (startTime + t)):
            print(self.getAccelerometerData())
        print("\n--------------------------------")
        print("Closing")


mpu = Accelerometer('9250', 2)
mpu.Accelerometer_Data(60)

