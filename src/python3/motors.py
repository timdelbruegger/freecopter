from Adafruit_PWM_Servo_Driver import PWM
import time

# set PWM frequency in Hz
# number of PWM signals per second: [40, 1000]
ADAFRUIT_PWM_FREQUENCY = 60

# i2c address of the adafruit PWM controller
ADAFRUIT_PWM_ADRESS = 0x40

PIN_MOTOR_1 = 0
PIN_MOTOR_2 = 4
PIN_MOTOR_3 = 8
PIN_MOTOR_4 = 12


# Used to control four brushless motors (quadrocopter)
# via Adafruit 16-Channel 12-bit PWM/Servo Driver - I2C interface - PCA9685
# See tutorial: https://learn.adafruit.com/adafruit-16-channel-servo-driver-with-raspberry-pi/library-reference
class AdafruitPwmControlledMotors:

    def __init__(self, debug_flag):
        self.pwmController = PWM(ADAFRUIT_PWM_ADRESS, debug = debug_flag)
        self.pwmController.setPWMFreq(ADAFRUIT_PWM_FREQUENCY)

        # These values must be chosen so that the ESC reacts to them.
        # TODO: maybe this has to change with pwm frequency??
        self.servoMin = 150  # min pulse length out of 4096
        self.servoMax = 600  # max pulse length out of 4096

    # channel: pin on PCA9685 where this motor is connected
    # motorSpeed: desired motor speed in [0.0, 1.0]
    def _servo_pulse(self, channel, motor_speed):

        assert motor_speed >= 0.0
        assert motor_speed <= 1.0

        # TODO: check formula!
        # pulse length in us (1.000.000 per second) per bit
#        pulseLength = 1000000 * self.pwmFrequency / 4096
#        pwmPulse = pulse * 1000 / pulseLength

        pwm_pulse = motor_speed * (self.servoMax - self.servoMin) + self.servoMin

        # setPWM(self, channel, on, off)
        # on: The tick (between 0..4095) when the signal should transition from low to high
        # off: The tick (between 0..4095) when the signal should transition from high to low
        # 4096 because of 12 bit resolution
        # we always start with ON at time 0
        self.pwmController.setPWM(channel, 0, pwm_pulse)

    # motorSpeeds: [0.0, 1.0]^4
    def setSpeed(self, motor_speeds):
        assert len(motor_speeds) == 4
        for i in range(0,4):
            assert 0.0 <= motor_speeds[i] <= 1.0

        self._servo_pulse(PIN_MOTOR_1, motor_speeds[0])
        self._servo_pulse(PIN_MOTOR_2, motor_speeds[1])
        self._servo_pulse(PIN_MOTOR_3, motor_speeds[2])
        self._servo_pulse(PIN_MOTOR_4, motor_speeds[3])
