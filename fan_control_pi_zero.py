
import spidev
import time
import gpiod
import threading
import subprocess
import RPi.GPIO as GPIO

# === CONFIGURATION ===
SPI_BUS = 0
SPI_DEVICE = 0
CS_GPIO = 5
FAN_PWM_GPIO = 18  # Hardware PWM pin
FAN_RPM_GPIO = 7

SET_POINT_F = 110.0

# === MAX31865 SETUP ===
spi = spidev.SpiDev()
spi.open(SPI_BUS, SPI_DEVICE)
spi.max_speed_hz = 500000

chip = gpiod.Chip('gpiochip0')
cs_line = chip.get_line(CS_GPIO)
cs_line.request(consumer='MAX31865', type=gpiod.LINE_REQ_DIR_OUT, default_vals=[1])

def max31865_read_temp():
    def write_register(reg, val):
        cs_line.set_value(0)
        spi.xfer2([0x80 | reg, val])
        cs_line.set_value(1)

    def read_registers(reg, length):
        cs_line.set_value(0)
        result = spi.xfer2([reg] + [0x00] * length)
        cs_line.set_value(1)
        return result[1:]

    write_register(0x00, 0xC2)  # Config register: VBIAS on, auto convert, 3-wire
    time.sleep(0.065)
    data = read_registers(0x01, 7)

    rtd_msb = data[0]
    rtd_lsb = data[1]
    rtd_raw = ((rtd_msb << 8) | rtd_lsb) >> 1

    RTD_RESISTANCE = rtd_raw * 400.0 / 32768.0
    RTD_NOMINAL = 100.0
    REF_RESISTOR = 430.0

    ratio = RTD_RESISTANCE / RTD_NOMINAL
    temp_c = (-242.02 + 2.2228 * RTD_RESISTANCE + 2.5859e-3 * RTD_RESISTANCE**2 - 4.8260e-6 * RTD_RESISTANCE**3)
    return temp_c * 9/5 + 32  # Return Fahrenheit

# === FAN PWM SETUP ===
def set_fan_duty(duty):
    duty = max(0, min(100, duty))
    subprocess.run(["pwm-ctl", "--pin", str(FAN_PWM_GPIO), "--freq", "25000", "--duty", str(duty)], check=False)

# === FAN RPM SETUP ===
rpm_count = 0
last_rpm_time = time.time()
lock = threading.Lock()

def rpm_callback(channel):
    global rpm_count
    with lock:
        rpm_count += 1

GPIO.setmode(GPIO.BCM)
GPIO.setup(FAN_RPM_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(FAN_RPM_GPIO, GPIO.FALLING, callback=rpm_callback)

# === MAIN LOOP ===
try:
    print("Starting Fan Control System on Raspberry Pi Zero 2 W")
    while True:
        temp_f = max31865_read_temp()
        print(f"Temperature: {temp_f:.2f} F")

        # Simple control: fan speed linearly ramps from 0% at 100°F to 100% at 130°F
        duty = max(0, min(100, (temp_f - 100) * 100 / 30))
        set_fan_duty(duty)

        # Calculate RPM every 2 seconds
        time.sleep(2)
        with lock:
            count = rpm_count
            rpm_count = 0
        rpm = (count / 2.0) * 30  # Assuming 1 pulse per revolution
        print(f"Fan RPM: {rpm:.0f}, Fan Duty: {duty:.0f}%\n")

except KeyboardInterrupt:
    print("Shutting down.")
finally:
    set_fan_duty(0)
    GPIO.cleanup()
    cs_line.set_value(1)
    spi.close()
