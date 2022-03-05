import RPi.GPIO as GPIO
from time import sleep

class MotorController:
    def __init__(self):
        # constants setup
        self.MOTOR_X_PIN = 12
        self.MOTOR_Y_PIN = 13
        self.PWM_FREQUENCY_HZ = 50
        self.AXES_LOWER_LIMIT = 0
        self.AXES_UPPER_LIMIT = 100
        self.AXES_RANGE = self.AXES_UPPER_LIMIT - self.AXES_LOWER_LIMIT
        self.ANGLE_LOWER_LIMIT = 0
        self.ANGLE_UPPER_LIMIT = 180
        self.ANGLE_RANGE = self.ANGLE_UPPER_LIMIT - self.ANGLE_LOWER_LIMIT
        self.MOTOR_MINIMUM_PULSE_S = 0.0004
        self.MOTOR_MAXIMUM_PULSE_S = 0.0020
        self.PWM_LOWER_LIMIT = (self.MOTOR_MINIMUM_PULSE_S / (1 / self.PWM_FREQUENCY_HZ)) * 100
        self.PWM_UPPER_LIMIT = (self.MOTOR_MAXIMUM_PULSE_S / (1 / self.PWM_FREQUENCY_HZ)) * 100
        self.PWM_RANGE = self.PWM_UPPER_LIMIT - self.PWM_LOWER_LIMIT
        self.PWM_MOVE_SLEEP_TIME_S = (1 / self.PWM_FREQUENCY_HZ) * 2

        # variable setup
        self.motorXPosition = None
        self.motorYPosition = None

        # disable GPIO warnings
        GPIO.setwarnings(False)

        # PWM setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.MOTOR_X_PIN, GPIO.OUT)
        GPIO.setup(self.MOTOR_Y_PIN, GPIO.OUT)
        self.motorXPwm = GPIO.PWM(self.MOTOR_X_PIN, self.PWM_FREQUENCY_HZ)
        self.motorYPwm = GPIO.PWM(self.MOTOR_Y_PIN, self.PWM_FREQUENCY_HZ)
        self.motorXPwm.start(0)
        self.motorYPwm.start(0)

        # move to centre
        centre = (self.AXES_RANGE / 2) + self.AXES_LOWER_LIMIT
        self.moveXY(centre, centre)

    def __del__(self):
        self.motorXPwm.stop()
        self.motorYPwm.stop()
        GPIO.cleanup()

    def moveXY(self, x: float, y: float):
        # check x and y values are in range
        assert(x >= self.AXES_LOWER_LIMIT)
        assert(y >= self.AXES_LOWER_LIMIT)
        assert(x <= self.AXES_UPPER_LIMIT)
        assert(y <= self.AXES_UPPER_LIMIT)

        # translate x and y to duty cycle values
        motorXPwmPercentage = (((x - self.AXES_LOWER_LIMIT) / self.AXES_RANGE) * self.PWM_RANGE) + self.PWM_LOWER_LIMIT
        motorYPwmPercentage = (((y - self.AXES_LOWER_LIMIT) / self.AXES_RANGE) * self.PWM_RANGE) + self.PWM_LOWER_LIMIT

        # move motors
        self._doMove(motorXPwmPercentage, motorYPwmPercentage)

    def moveAngles(self, xAngle: float, yAngle: float):
        # check angle values are in range
        assert(xAngle >= self.ANGLE_LOWER_LIMIT)
        assert(yAngle >= self.ANGLE_LOWER_LIMIT)
        assert(xAngle <= self.ANGLE_UPPER_LIMIT)
        assert(yAngle <= self.ANGLE_UPPER_LIMIT)

        # translate angles to duty cycle values
        motorXPwmPercentage = (((xAngle - self.ANGLE_LOWER_LIMIT) / self.ANGLE_RANGE) * self.PWM_RANGE) + self.PWM_LOWER_LIMIT
        motorYPwmPercentage = 0

        # move motors
        self._doMove(motorXPwmPercentage, motorYPwmPercentage)

    def getXY(self) -> tuple[float, float]:
        xVal = (((self.motorXPosition - self.PWM_LOWER_LIMIT) / self.PWM_RANGE) * self.AXES_RANGE) + self.AXES_LOWER_LIMIT
        yVal = (((self.motorYPosition - self.PWM_LOWER_LIMIT) / self.PWM_RANGE) * self.AXES_RANGE) + self.AXES_LOWER_LIMIT

        return (xVal, yVal)

    def getAngles(self) -> tuple[float, float]:
        xVal = (((self.motorXPosition - self.PWM_LOWER_LIMIT) / self.PWM_RANGE) * self.ANGLE_RANGE) + self.ANGLE_LOWER_LIMIT
        yVal = (((self.motorYPosition - self.PWM_LOWER_LIMIT) / self.PWM_RANGE) * self.ANGLE_RANGE) + self.ANGLE_LOWER_LIMIT

        return (xVal, yVal)

    def _doMove(self, xDutyCycle: float, yDutyCycle: float):
        # record motor positions
        self.motorXPosition = xDutyCycle
        self.motorYPosition = yDutyCycle

        # set duty cycle
        self.motorXPwm.ChangeDutyCycle(xDutyCycle)
        self.motorYPwm.ChangeDutyCycle(yDutyCycle)

        # wait for move to complete
        sleep(self.PWM_MOVE_SLEEP_TIME_S)

# test script
if __name__ == "__main__":
    # create an instance of the MotorController class
    controller = MotorController()

    # move the motors from one end of their range to the other
    for i in range(0, 101):
        controller.moveXY(i, i)
        sleep(1)
