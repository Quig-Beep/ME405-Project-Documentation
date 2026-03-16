import pyb
from pyb import I2C
from micropython import const
from time import sleep_ms
import os

# --- I2C address options ---
ADDR_A = const(0x28)
ADDR_B = const(0x29)

# --- Registers (subset) ---
CHIP_ID     = const(0x00)
PAGE_ID     = const(0x07)

ACC_DATA_X_LSB   = const(0x08)
MAG_DATA_X_LSB   = const(0x0E)
GYR_DATA_X_LSB   = const(0x14)
EULER_H_LSB      = const(0x1A)
QUA_W_LSB        = const(0x20)
LIA_DATA_X_LSB   = const(0x28)
GRV_DATA_X_LSB   = const(0x2E)

ACC_OFF_X_LSB = const(0x55)
MAG_OFF_X_LSB = const(0x5B)
GYR_OFF_X_LSB = const(0x61)
ACC_RAD_LSB = const(0x67)

CALIB_STAT  = const(0x35)

OPR_MODE    = const(0x3D)
PWR_MODE    = const(0x3E)
SYS_TRIGGER = const(0x3F)
UNIT_SEL    = const(0x3B)

# --- Operation modes ---
MODE_CONFIG = const(0x00)
MODE_NDOF   = const(0x0C)   # fusion: accel+gyro+mag
MODE_IMU    = const(0x08)

# --- Power modes ---
POWER_NORMAL = const(0x00)

class IMU:
    def __init__(self, i2c, addr=ADDR_A):
        # Using I2C2, device address is 0x28
        self._i2c = i2c
        self._addr = addr
        self._buf = bytearray(8)

    def readbyte(self, reg):
        return self._i2c.mem_read(1, self._addr, reg)[0]
    def readbytes(self, n, reg):
        return self._i2c.mem_read(n, self._addr, reg)
    def writebytes(self, val, reg):
        self._i2c.mem_write(bytes([val & 0xFF]), self._addr, reg)
    def write2bytes(self, val, reg):
        val &= 0xFFFF
        lsb = val&0xFF
        msb = (val >> 8) & 0xFF
        self.writebytes(lsb,reg)
        self.writebytes(msb,reg+1)
    @staticmethod
    def combine(lsb, msb):
        v = (msb<<8) | lsb
        return v-65536 if v & 0x8000 else v
    
    def read_vector(self, startreg, unit_scale):
        data = self.readbytes(6, startreg)
        x = self.combine(data[0], data[1]) / unit_scale
        y = self.combine(data[2], data[3]) / unit_scale
        z = self.combine(data[4], data[5]) / unit_scale
        return x,y,z
    def set_mode(self, mode):
        self.writebytes(mode, OPR_MODE)
        sleep_ms(30)
    def initialize(self, mode=MODE_IMU, reset=True, units = 0x06):
        # units 0x00 gives m/s^2, degrees/sec, Celcius
        self.writebytes(0x00, PAGE_ID)
        sleep_ms(5)
        self.set_mode(MODE_CONFIG)
        # Power Mode: Normal
        self.writebytes(POWER_NORMAL, PWR_MODE)
        sleep_ms(20)
        self.writebytes(units, UNIT_SEL) # select standard units
        sleep_ms(10)
        # Calibration data
        fname = "calibration.txt"
        try:
            os.stat(fname)
            with open(fname, 'r') as file:
                constants = file.read().strip().split(',')
                vals = [int(c) for c in constants if c!=""]
                print(len(vals))
                ACC = vals[0:3]
                MAG = vals[3:6]
                GYR = vals[6:9]
                print(vals)
                ACC_RAD = vals[9]
                MAG_RAD = vals[10]
                self.write_coeff(ACC, MAG, GYR, ACC_RAD, MAG_RAD)
        except OSError:
                self.set_mode(MODE_NDOF)
                while(True):
                    stat = self.calibration_status()
                    print(stat)
                    if all(bit == 3 for bit in stat[1:]):
                        print("Calibration complete")
                        break
                self.set_mode(MODE_CONFIG)
                ACC_OFF, MAG_OFF, GYR_OFF, ACC_RAD, MAG_RAD = self.get_coeff()
                lst = list(ACC_OFF) + list(MAG_OFF) + list(GYR_OFF) + [ACC_RAD, MAG_RAD]
                with open(fname, 'w') as file:
                    file.write(','.join(str(c) for c in lst)+ "\n")
        self.set_mode(mode) # enter IMU mode
    def calibration_status(self, reg=CALIB_STAT):
        # data in the order: system, gyro, accelerometer, magnetometer
        cbt = self.readbyte(reg)
        system = (cbt >> 6) & 0b11
        gyro = (cbt >> 4) & 0b11
        acc = (cbt >> 2) & 0b11
        mag = cbt & 0b11
        return [system, gyro, acc, mag]
    def get_coeff(self,reg=ACC_OFF_X_LSB):
        # format: [x data, y data, z data]
        raw = self.readbytes(22, reg)
        ACC_OFF = (self.combine(raw[0],raw[1]) & 0xFFFF,self.combine(raw[2],raw[3]) & 0xFFFF,self.combine(raw[4],raw[5]) & 0xFFFF)
        MAG_OFF = (self.combine(raw[6],raw[7]) & 0xFFFF,self.combine(raw[8],raw[9]) & 0xFFFF,self.combine(raw[10],raw[11]) & 0xFFFF)
        GYR_OFF = (self.combine(raw[12],raw[13]) & 0xFFFF,self.combine(raw[14],raw[15]) & 0xFFFF,self.combine(raw[16],raw[17]) & 0xFFFF)
        ACC_RAD = self.combine(raw[18],raw[19]) & 0xFFFF
        MAG_RAD = self.combine(raw[20],raw[21]) & 0xFFFF
        return ACC_OFF,MAG_OFF,GYR_OFF,ACC_RAD,MAG_RAD
    def write_coeff(self, ACC, MAG, GYR, ACC_RAD, MAG_RAD, reg=ACC_OFF_X_LSB):
        self.write2bytes(ACC[0], reg)
        self.write2bytes(ACC[1], reg+2)
        self.write2bytes(ACC[2], reg+4)
        self.write2bytes(MAG[0], reg+6)
        self.write2bytes(MAG[1], reg+8)
        self.write2bytes(MAG[2], reg+10)
        self.write2bytes(GYR[0], reg+12)
        self.write2bytes(GYR[1], reg+14)
        self.write2bytes(GYR[2], reg+16)
        self.write2bytes(ACC_RAD, reg+18)
        self.write2bytes(MAG_RAD, reg+20)
    def euler(self, reg=EULER_H_LSB):
        angles = self.read_vector(reg, 900.0)
        return angles
    def ang_v(self, reg=GYR_DATA_X_LSB):
        rates = self.read_vector(reg, 900.0)
        return rates
    def accel(self, reg=ACC_DATA_X_LSB):
        accel = self.read_vector(reg, 100)
        return(accel)
