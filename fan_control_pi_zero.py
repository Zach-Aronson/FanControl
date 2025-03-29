
import spidev
import time
import gpiod
import threading
import RPi.GPIO as GPIO

# === CONFIGURATION ===
SPI_BUS = 0
SPI_DEVICE = 0
CS_GPIO = 5
FAN_PWM_GPIO = 18  # Must be hardware PWM-capable
FAN_RPM_GPIO = 7   # BCM GPIO number

SET_POINT_F = 110.0

# === MAX31865 SETUP ===
spi = spidev.SpiDev()
spi.open(SPI_BUS, SPI_DEVICE)
spi.max_speed_hz = 500000

chip = gpiod.Chip('gpiochip0')
cs_line = chip.get_line(CS_GPIO)
cs_line.request(consumer='MAX31865_CS', type=gpiod.LINE_REQ_DIR_OUT, default_vals=[1])

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

    write_register(0x00, 0xC2)
    time.sleep(0.065)
    data = read_registers(0x01, 7)

    rtd_msb = data[0]
    rtd_lsb = data[1]
    rtd_raw = ((rtd_msb << 8) | rtd_lsb) >> 1

    RTD_RESISTANCE = rtd_raw * 400.0 / 32768.0
    RTD_NOMINAL = 100.0
    temp_c = (-242.02 + 2.2228 * RTD_RESISTANCE + 2.5859e-3 * RTD_RESISTANCE**2 - 4.8260e-6 * RTD_RESISTANCE**3)
    return temp_c * 9/5 + 32

# === FAN PWM SETUP (using RPi.GPIO) ===
GPIO.setmode(GPIO.BCM)
GPIO.setup(FAN_PWM_GPIO, GPIO.OUT)
pwm = GPIO.PWM(FAN_PWM_GPIO, 25000)
pwm.start(0)

def set_fan_duty(duty):
    duty = max(0, min(100, duty))
    pwm.ChangeDutyCycle(duty)

# === FAN RPM SETUP USING GPIOD ===
rpm_count = 0
lock = threading.Lock()

def rpm_event_callback(line, event_type):
    global rpm_count
    with lock:
        rpm_count += 1

def setup_rpm_gpio():
    rpm_chip = gpiod.Chip("gpiochip0")
    rpm_line = rpm_chip.get_line(FAN_RPM_GPIO)
    rpm_line.request(consumer="RPM_INPUT", type=gpiod.LINE_REQ_EV_FALLING_EDGE)
    return rpm_line

# === MAIN LOOP ===
try:
    print("Starting Fan Control System with gpiod RPM + RPi.GPIO PWM")
    rpm_line = setup_rpm_gpio()

    while True:
        temp_f = max31865_read_temp()
        print(f"Temperature: {temp_f:.2f} F")

        duty = max(0, min(100, (temp_f - 100) * 100 / 30))
        set_fan_duty(duty)

        start_time = time.time()
        with lock:
            rpm_count = 0
        while time.time() - start_time < 2:
            if rpm_line.event_wait(sec=1):
                event = rpm_line.event_read()
                rpm_event_callback(rpm_line, event.event_type)

        with lock:
            count = rpm_count
        rpm = (count / 2.0) * 30
        print(f"Fan RPM: {rpm:.0f}, Fan Duty: {duty:.0f}%\n")

except KeyboardInterrupt:
    print("Shutting down.")
finally:
    set_fan_duty(0)
    pwm.stop()
    GPIO.cleanup()
    spi.close()
