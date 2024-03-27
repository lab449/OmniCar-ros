import VL53L0X
import RPi.GPIO as GPIO
import time
import numpy as np

I2C_DEFAULT_ADRESS = 0x29

class RangeSensorDriver:
    def __init__(self, shutdown_pins: list) -> None:
        self.__shutdown_pins = shutdown_pins
        self.__i2c_adresses = []
        for i in range(0, len(shutdown_pins)):
            self.__i2c_adresses[i].append(I2C_DEFAULT_ADRESS+i+1)
        self.__sensors = []
        self.__timing = []
        assert len(self.__shutdown_pins), "Count of shutdown pins can not be 0"
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        for p in self.__shutdown_pins:
            GPIO.setup(p, GPIO.OUT)
            GPIO.setup(p, GPIO.OUT)

            # Set all shutdown pins low to turn off each VL53L0X
            GPIO.output(p, GPIO.LOW)
            GPIO.output(p, GPIO.LOW)

        # Keep all low for 500 ms or so to make sure they reset
        time.sleep(0.50)

    def init(self):
        self.__sensors.clear()
        for i in range(0, len(self.__shutdown_pins)):
            tof = VL53L0X.VL53L0X(i2c_bus=1,i2c_address=I2C_DEFAULT_ADRESS)
            self.__sensors.append(tof)
            # I2C Address can change before tof.open()
            tof.change_address(self.__i2c_adresses[i])
            tof.open()
            GPIO.output(self.__i2c_adresses[i], GPIO.HIGH)
            time.sleep(0.50)
            GPIO.output(self.__i2c_adresses[i], GPIO.LOW)
            time.sleep(0.50)
        
        for i in range(0, len(self.__shutdown_pins)):
            GPIO.output(self.__i2c_adresses[i], GPIO.HIGH)
            time.sleep(0.50)

    def start(self, mode: int = VL53L0X.Vl53l0xAccuracyMode.BETTER):
        self.__timing.clear()
        for i in range(0, len(self.self.__i2c_adresses)):
            self.__sensors[i].start_ranging(mode)
            self.__timing[i] = self.__sensors[i].get_timing()
            if self.__timing[i] < 20000:
                self.__timing[i] = 20000
            print("Timing %d ms" % (self.__timing[i]/1000))
    
    def stop(self):
        for i in range(0, len(self.__i2c_adresses)):
            self.__sensors[i].stop_ranging()
    
    def close(self):
        for i in range(0, len(self.__i2c_adresses)):
            self.__sensors[i].close()
    
    def get_data(self) -> np.ndarray:
        data = []
        for i in range(0, len(self.__i2c_adresses)):
            data.append(self.__sensors[i].get_distance())
            time.sleep(self.__timing[i]/1000000.00)
        return np.array(data)