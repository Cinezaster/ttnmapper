from machine import Timer
import time
import gc
import binascii

class L76GNSS:

    GPS_I2CADDR = const(0x10)

    def __init__(self, pytrack=None, sda='P22', scl='P21', timeout=None):
        if pytrack is not None:
            self.i2c = pytrack.i2c
        else:
            from machine import I2C
            self.i2c = I2C(0, mode=I2C.MASTER, pins=(sda, scl))

        self.chrono = Timer.Chrono()

        self.timeout = timeout
        self.timeout_status = True

        self.reg = bytearray(1)
        self.i2c.writeto(GPS_I2CADDR, self.reg)

        self.lat = 0
        self.lng = 0
        self.hdop = 0
        self.alt = 0

    def _read(self):
        self.reg = self.i2c.readfrom(GPS_I2CADDR, 64)
        return self.reg

    def _convert_gll_coords(self, gngll_s):
        lat = gngll_s[1]
        lat_d = (float(lat) // 100) + ((float(lat) % 100) / 60)
        lon = gngll_s[3]
        lon_d = (float(lon) // 100) + ((float(lon) % 100) / 60)
        if gngll_s[2] == 'S':
            lat_d *= -1
        if gngll_s[4] == 'W':
            lon_d *= -1
        self.lat = lat_d
        self.lng = lon_d
    def _convert_rmc_coords(self, gngll_s):
        lat = gngll_s[3]
        lat_d = (float(lat) // 100) + ((float(lat) % 100) / 60)
        lon = gngll_s[5]
        lon_d = (float(lon) // 100) + ((float(lon) % 100) / 60)
        if gngll_s[4] == 'S':
            lat_d *= -1
        if gngll_s[6] == 'W':
            lon_d *= -1
        self.lat = lat_d
        self.lng = lon_d

    def coordinates(self, debug=False):
        lat_d, lon_d, debug_timeout = None, None, False
        if self.timeout != None:
            self.chrono.start()
        nmea = b''
        while True:
            if self.timeout != None and self.chrono.read() >= self.timeout:
                self.chrono.stop()
                chrono_timeout = self.chrono.read()
                self.chrono.reset()
                self.timeout_status = False
                debug_timeout = True
            if self.timeout_status != True:
                gc.collect()
                break
            nmea += self._read().lstrip(b'\n\n').rstrip(b'\n\n')
            startIndex = nmea.find(b'$G')
            if startIndex >= 0:
                nmeastr = nmea[startIndex+1:]
                endIndex = nmeastr.find(b'\r\n')
                if endIndex >= 0:
                    try:
                        nmeastr = nmeastr[:endIndex].decode('ascii')
                        nmea_s = nmeastr.split(',')
                        if nmea_s[0].endswith("RMC"):
                            self._convert_rmc_coords(nmea_s)
                        elif nmea_s[0].endswith("GGA"):
                            self.hdop = float(nmea_s[8])
                            self.alt = float(nmea_s[9])
                        elif nmea_s[0].endswith("GLL"):
                            self._convert_gll_coords(nmea_s)
                        elif nmea_s[0].endswith("GSA"):
                            self.hdop = float(nmea_s[16])
                    except Exception:
                        pass
                    finally:
                        nmea = nmea[(startIndex + endIndex):]
                        gc.collect()
                        break
            else:
                gc.collect()
                if len(nmea) > 4096:
                    nmea = b''
            time.sleep(0.1)
        self.timeout_status = True
        if debug and debug_timeout:
            print('GPS timed out after %f seconds' % (chrono_timeout))
            return(None, None)
        else:
            return(self.lat, self.lng, self.hdop, self.alt)
