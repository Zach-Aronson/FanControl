
import time
import board
import busio
import digitalio
import adafruit_max31865
import pigpio
import gpiod
import threading

# === CONFIGURATION ===
FAN_PWM_GPIO = 18  # GPIO18 (Pin 12)
FAN_RPM_GPIO = 17  # GPIO17 (Pin 11)
CS_GPIO = 5        # GPIO5 (Pin 29) for MAX31865
SET_POINT_F = 110.0

# === Setup SPI and MAX31865 using Adafruit library ===
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
cs = digitalio.DigitalInOut(board.D5)  # Match to GPIO5
sensor = adafruit_max31865.MAX31865(spi, cs, wires=3)

# === Setup pigpio for PWM ===
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("Could not connect to pigpio daemon")

def set_fan_duty(duty):
    duty = max(0, min(100, duty))
    frequency = 25000
    duty_cycle = int(1_000_000 * duty / 100)
    pi.hardware_PWM(FAN_PWM_GPIO, frequency, duty_cycle)

# === Setup gpiod for RPM sensing ===
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
    print("Starting Fan Control System with Adafruit MAX31865 library")
    rpm_line = setup_rpm_gpio()

    while True:
        try:
            temp_c = sensor.temperature
            temp_f = temp_c * 9 / 5 + 32
            print(f"Temperature: {temp_f:.2f} F")
        except Exception as e:
            print(f"Sensor error: {e}")
            temp_f = -459.67  # Force safe temp on error

        # Fan control logic
        duty = max(0, min(100, (temp_f - 100) * 100 / 30))
        set_fan_duty(duty)

        # Measure RPM
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
    pi.hardware_PWM(FAN_PWM_GPIO, 0, 0)
    pi.stop()
