
class QMI8658(object):
    def __init__(self, smbus=2, address=0X6B):
        self._address = address
        import smbus2
        self._bus = smbus2.SMBus(smbus)
        bRet = self.WhoAmI()
        if bRet:
            self.Read_Revision()
        else:
            return None
        self.Config_apply()

    def _read_byte(self, cmd):
        rec = self._bus.read_i2c_block_data(int(self._address), int(cmd), 1)
        return rec[0]

    def _read_block(self, reg, length=1):
        rec = self._bus.read_i2c_block_data(int(self._address), int(reg), length)
        return rec

    def _read_u16(self, cmd):
        LSB = self._bus.read_i2c_block_data(int(self._address), int(cmd), 1)
        MSB = self._bus.read_i2c_block_data(int(self._address), int(cmd)+1, 1)
        return (MSB[0] << 8) + LSB[0]

    def _write_byte(self, cmd, val):
        self._bus.write_i2c_block_data(int(self._address), int(cmd), bytes([int(val)]))

    def WhoAmI(self):
        bRet = False
        if (0x05) == self._read_byte(0x00):
            bRet = True
        return bRet

    def Read_Revision(self):
        return self._read_byte(0x01)

    def Config_apply(self):
        # REG CTRL1
        self._write_byte(0x02, 0x60)
        # REG CTRL2 : QMI8658AccRange_8g  and QMI8658AccOdr_1000Hz
        self._write_byte(0x03, 0x23)
        # REG CTRL3 : QMI8658GyrRange_512dps and QMI8658GyrOdr_1000Hz
        self._write_byte(0x04, 0x53)
        # REG CTRL4 : No
        self._write_byte(0x05, 0x00)
        # REG CTRL5 : Enable Gyroscope And Accelerometer Low-Pass Filter
        self._write_byte(0x06, 0x11)
        # REG CTRL6 : Disables Motion on Demand.
        self._write_byte(0x07, 0x00)
        # REG CTRL7 : Enable Gyroscope And Accelerometer
        self._write_byte(0x08, 0x03)

    def Read_Raw_XYZ(self):
        xyz = [0, 0, 0, 0, 0, 0]
        raw_timestamp = self._read_block(0x30, 3)
        raw_acc_xyz = self._read_block(0x35, 6)
        raw_gyro_xyz = self._read_block(0x3b, 6)
        raw_xyz = self._read_block(0x35, 12)
        timestamp = (raw_timestamp[2] << 16) | (
            raw_timestamp[1] << 8) | (raw_timestamp[0])
        for i in range(6):
            # xyz[i]=(raw_acc_xyz[(i*2)+1]<<8)|(raw_acc_xyz[i*2])
            # xyz[i+3]=(raw_gyro_xyz[((i+3)*2)+1]<<8)|(raw_gyro_xyz[(i+3)*2])
            xyz[i] = (raw_xyz[(i*2)+1] << 8) | (raw_xyz[i*2])
            if xyz[i] >= 32767:
                xyz[i] = xyz[i]-65535
        return xyz

    def Read_XYZ(self):
        xyz = [0, 0, 0, 0, 0, 0]
        raw_xyz = self.Read_Raw_XYZ()
        #QMI8658AccRange_8g
        acc_lsb_div = (1 << 12)
        #QMI8658GyrRange_512dps
        gyro_lsb_div = 64
        for i in range(3):
            xyz[i] = raw_xyz[i]/acc_lsb_div  # (acc_lsb_div/1000.0)
            xyz[i+3] = raw_xyz[i+3]*1.0/gyro_lsb_div
        return xyz


if __name__=='__main__':

    qmi8658=QMI8658()

    while(True):
        #read QMI8658
        xyz=qmi8658.Read_XYZ()

        print("ACC_X={:+.2f}".format(xyz[0]))
        print("ACC_Y={:+.2f}".format(xyz[1]))
        print("ACC_Z={:+.2f}".format(xyz[2]))

        print("GYR_X={:+3.2f}".format(xyz[3]))
        print("GYR_Y={:+3.2f}".format(xyz[4]))
        print("GYR_Z={:+3.2f}".format(xyz[5]))
        import time
        time.sleep(0.1)
