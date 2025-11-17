from vex import *
from UI import *

class PID:
    """Generic PID controller for reading a sensor and computing an output.

    Attributes:
        yourSensor: callable returning current sensor value (e.g., gyro.heading)
        brain: Brain instance (used for SD card, screen, etc.)
        KP, KI, KD: PID constants
        output: last computed output value
        stopButton: if True, tune() will show an on-screen stop button
    """

    def __init__(self, yourSensor, brain: Brain, KP: float = 1, KI: float = 0, KD: float = 0):
        self.KP = KP
        self.KI = KI
        self.KD = KD
        self.yourSensor = yourSensor
        self.brain = brain
        self.output: float = 0

    def run(self, desiredValue: int, tollerance: float):
        """Run PID loop until the sensor reaches desiredValue within tolerance.

        This method updates self.output. It does not apply the output to motors —
        subclasses or callers should use self.output as required.
        """
        previousError = 0
        totalError = 0
        i = 0
        while abs(desiredValue - self.yourSensor()) > tollerance:
            i += 1
            error = desiredValue - self.yourSensor()
            derivative = (error - previousError) / (i*50)
            totalError += error
            self.output = error * self.KP + derivative * self.KD + (totalError * (i*50)) * self.KI
            wait(50)
            previousError = error

    def tune(self, desiredValue: int, tollerance: float, sd_file_name = "pidData.csv", stopButton = False):
        """Run PID loop and save tuning data to SD card.

        Produces CSV with columns:
            time, error, derivative, totalError, output, desiredValue

        If stopButton is True, displays a red 'terminate' button on the brain screen
        allowing the operator to abort and save partial data.
        """
        if stopButton:
            stop = button(60, 220, 250, 10, Color.RED, "terminate")
            stop.draw()

        csvHeaderText = "time, error, derivative, totalError, output, desiredValue"
        data_buffer = csvHeaderText + "\n"

        previousError = 0
        totalError = 0
        i = 0

        while abs(desiredValue - self.yourSensor()) > tollerance:
            i += 1
            error = desiredValue - self.yourSensor()
            derivative = (error - previousError) / (i*50)
            totalError += error
            self.output = error * self.KP + derivative * self.KD + (totalError * (i*50)) * self.KI
            wait(50)
            previousError = error

            # append one row of data to buffer
            data_buffer +=  str(i * 50) + ","
            data_buffer += "%.3f" % error + ","
            data_buffer += "%.3f" % derivative + ","
            data_buffer += "%.3f" % (totalError * (i*50)) + ","
            data_buffer += "%.3f" % self.output + ","
            data_buffer += str(desiredValue) + "\n"
            
            # allow user to abort when using touchscreen stop button
            if stopButton and stop.isPressed(self.brain.screen.x_position(),self.brain.screen.y_position()):
                break

        # save CSV to SD card (brain.sdcard)
        self.brain.sdcard.savefile(sd_file_name, bytearray(data_buffer, 'utf-8'))

class turnPID(PID):
    """PID controller specialized for turning a drivetrain (left/right motor groups).

    It computes a rotational output and sets velocities on left and right MotorGroups.

    Constructor parameters:
        yourSensor: callable returning current heading/angle
        brain: Brain instance
        leftMotorGroup, rightMotorGroup: MotorGroup instances to apply rotation
        KP, KI, KD: PID gains
        stopButton: enable touchscreen terminate button during tune()
    """

    def __init__(self, yourSensor, brain: Brain, leftMotorGroup: MotorGroup, rightMotorGroup: MotorGroup, KP: float = 1, KI: float = 0, KD: float = 0):
        self.KP = KP
        self.KI = KI
        self.KD = KD
        self.left = leftMotorGroup
        self.right = rightMotorGroup
        self.yourSensor = yourSensor
        self.brain = brain
        self.output:float = 0

    def run (self, desiredValue: int, tollerance: float):
        """Run turn PID and set motor velocities until target heading reached."""
        previousError = 0
        totalError = 0
        i = 0
        while abs(desiredValue - self.yourSensor()) > tollerance:
            i += 1
            error:float = desiredValue - self.yourSensor() if desiredValue - self.yourSensor() <= 180 else desiredValue - self.yourSensor() - 180
            derivative = (error - previousError) / (i*50)
            totalError += error
            self.output = error * self.KP + derivative * self.KD + (totalError * (i*50)) * self.KI
            # set velocities such that robot rotates in place
            self.left.set_velocity(self.output, PERCENT)
            self.right.set_velocity(-self.output, PERCENT)
            wait(50)
            previousError = error

    def tune(self, desiredValue: int, tollerance: float, sd_file_name = "pidData.csv", stopButton = False):
        """Run tuning loop similar to PID.tune but apply outputs to drivetrain and save CSV.

        CSV columns:
            time, proportional, derivative, integral, output, desiredValue
        """
        if stopButton:
            stop = button(60, 220, 250, 10, Color.RED, "terminate")
            stop.draw()

        csvHeaderText:str = "time, proportional, derivative, integral, output, desiredValue"
        data_buffer:str = csvHeaderText + "\n"

        previousError = 0
        totalError = 0
        i = 0

        while abs(desiredValue - self.yourSensor()) > tollerance:
            i += 1
            error:float = desiredValue - self.yourSensor() if desiredValue - self.yourSensor() <= 180 else desiredValue - self.yourSensor() - 180
            derivative = (error - previousError) / (i*50)
            totalError += error
            self.output = error * self.KP + derivative * self.KD + (totalError * (i*50)) * self.KI
            self.left.set_velocity(self.output, PERCENT)
            self.right.set_velocity(-self.output, PERCENT)
            wait(50)
            previousError = error

            # save one row of data
            data_buffer += str(i * 50) + ","
            data_buffer += "%.3f" % (error*self.KP) + ","
            data_buffer += "%.3f" % (derivative * self.KD)  + ","
            data_buffer += "%.3f" % (totalError * (i*50)*self.KI) + ","
            data_buffer += "%.3f" % self.output + ","
            data_buffer += str(desiredValue) + "\n"

            if stopButton and stop.isPressed(self.brain.screen.x_position(),self.brain.screen.y_position()):
                break

        self.brain.sdcard.savefile(sd_file_name, bytearray(data_buffer, 'utf-8'))
