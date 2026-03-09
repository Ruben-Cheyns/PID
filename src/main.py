# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       RubenCheyns                                                  #
# 	Created:      4/28/2025, 1:04:28 PM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

#-------------------#
# vex device config #
#-------------------#
brain = Brain()
gyro = Inertial(Ports.PORT17)
controller_1 = Controller()
controller_2 = Controller()

left_1 = Motor(Ports.PORT20, GearSetting.RATIO_6_1, False)
left_2 = Motor(Ports.PORT19, GearSetting.RATIO_6_1, False)
left_3 = Motor(Ports.PORT18, GearSetting.RATIO_6_1, True)
left = MotorGroup(left_1, left_2, left_3)

right_1 = Motor(Ports.PORT10, GearSetting.RATIO_6_1, True)
right_2 = Motor(Ports.PORT9, GearSetting.RATIO_6_1, True)
right_3 = Motor(Ports.PORT8, GearSetting.RATIO_6_1, False)
right = MotorGroup(right_1, right_2, right_3)

intakeMotor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, True)
storageMotor = Motor(Ports.PORT11, GearSetting.RATIO_18_1, True)
outMotor = Motor(Ports.PORT16, True)

loaderPiston = Pneumatics(brain.three_wire_port.a)
descorePiston = Pneumatics(brain.three_wire_port.h)
outPiston = Pneumatics(brain.three_wire_port.b)

#-------------#
# PID classes #
#-------------#
class PID:
    """Generic PID controller for reading a sensor and computing an output.

    Attributes:
        yourSensor: callable returning current sensor value (e.g., gyro.heading)
        brain: Brain instance (used for SD card, screen, etc.)
        KP, KI, KD: PID constants
        output: last computed output value
        stopButton: if True, graph() will show an on-screen stop button
    """

    def __init__(self, yourSensor, brain: Brain, KP: float = 1, KI: float = 0, KD: float = 0):
        self.KP = KP
        self.KI = KI
        self.KD = KD
        self.yourSensor = yourSensor
        self.brain = brain
        self.output: float = 0

    def run(self, desiredValue: int, tolerance: float):
        """Run PID loop until the sensor reaches desiredValue within tolerance.

        This method updates self.output. It does not apply the output to motors —
        subclasses or callers should use self.output as required.
        """
        previousError = 0
        totalError = 0
        i = 0
        while abs(desiredValue - self.yourSensor()) > tolerance:
            i += 1
            error = desiredValue - self.yourSensor()
            derivative = (error - previousError) / (i*50)
            totalError += error
            self.output = error * self.KP + derivative * self.KD + (totalError * (i*50)) * self.KI
            wait(50)
            previousError = error

    def graph(self, desiredValue: int, tolerance: float, sd_file_name = "pidData.csv", stopButton = False):
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

        while abs(desiredValue - self.yourSensor()) > tolerance:
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
        speedCap: a maximal value at wich the controller stops working and outputs this fixed value
        KP, KI, KD: PID gains
    """

    def __init__(self, yourSensor, brain: Brain, leftMotorGroup: MotorGroup, rightMotorGroup: MotorGroup, speedCap: int = 100, KP: float = 1, KI: float = 0, KD: float = 0):
        self.KP = KP
        self.KI = KI
        self.KD = KD
        self.left = leftMotorGroup
        self.right = rightMotorGroup
        self.yourSensor = yourSensor
        self.brain = brain
        self.output:float = 0
        self.speedCap:int = speedCap

    def run (self, desiredValue: int, tolerance: float, settleTime: float = 0.5):
        """Run turn PID and set motor velocities until target heading stabilized.
        desiredValue: the angle you wish to turn to in degrees
        tolerance: the tolerance used to detect setpoint stabilization in degrees
        settleTime: how long the setpoint must be stable to exit the pid loop in seconds
        """
        # starts up drivetrain motors with speed set to zero
        self.right.spin(FORWARD, 0)
        self.left.spin(FORWARD, 0)

        # necessary local variables, totalError over the entire loop and i: iterations
        totalError:float = 0.0
        i = 0

        # calculates the initial error, paying mind to the smallest angles.
        if desiredValue - self.yourSensor() > 0:
            if desiredValue - self.yourSensor() <= 180:
                error:float = desiredValue - self.yourSensor()
            elif desiredValue - self.yourSensor() > 180:
                error:float = desiredValue - self.yourSensor() - 360
        else:
            if desiredValue - self.yourSensor() >= -180:
                error:float = desiredValue - self.yourSensor()
            elif desiredValue - self.yourSensor() < -180:
                error:float = 360 + (desiredValue - self.yourSensor())

        # adds initial error to the errorList used in detecting stabilization and previousError used in the derivative
        errorList = [error]
        previousError:float = error

        # runs the PID loop until setpoint is stabilized by detecting the absolute smallest error
        while abs(max(errorList, key=abs)) > tolerance:
            # counts iterations
            i += 1
            #calculates error, paying mind to smallest angles
            if desiredValue - self.yourSensor() > 0:
                if desiredValue - self.yourSensor() <= 180:
                    error:float = desiredValue - self.yourSensor()
                elif desiredValue - self.yourSensor() > 180:
                    error:float = desiredValue - self.yourSensor() - 360
            else:
                if desiredValue - self.yourSensor() >= -180:
                    error:float = desiredValue - self.yourSensor()
                elif desiredValue - self.yourSensor() < -180:
                    error:float = 360 + (desiredValue - self.yourSensor())

            # calculates derivative using error, previousError and sampling time 
            derivative = (error - previousError) / 0.050
            # adds current error to totalError used in integral term
            totalError += error
            # sets output to the total PID equation or the speedCap
            self.output = min(error * self.KP + derivative * self.KD + (totalError * 0.050) * self.KI, (self.speedCap if error * self.KP + derivative * self.KD + (totalError * 0.050) * self.KI > 0 else -self.speedCap), key=abs)
            # sets totalError to zero if speedCap reached to prevent integral windup 
            if abs(self.output) == self.speedCap:
                totalError = 0
            # sets motor velocity to the PID output
            self.left.set_velocity(self.output, PERCENT)
            self.right.set_velocity(-self.output, PERCENT)
            # waits 50 MS to lighten program load
            wait(50)
            # sets previousError to current error for derivative term 
            previousError = error
            # adds error to errorList and deletes the first error if the list is longer than the settleTime divided by sampling time
            errorList.append(error)
            if len(errorList) > settleTime/0.050:
                errorList.pop(0)

    def graph(self, desiredValue: int, tolerance: float, settleTime: float = 0.5, sd_file_name = "pidData.csv", stopButton = False):
        """Runs loop similar to PID.run but saves a CSV containing PID data.

        CSV columns:
            time, proportional, derivative, integral, output, desiredValue, angle
        """
        # makes a stopButton if set to true to manually stop the loop
        if stopButton:
            stop = button(60, 220, 250, 10, Color.RED, "terminate")
            stop.draw()
            brain.screen.render()

        # csv variables to store PID data
        csvHeaderText:str = "time, proportional, derivative, integral, output, desiredValue, angle"
        data_buffer:str = csvHeaderText + "\n"

        # starts up drivetrain motors with speed set to zero
        self.right.spin(FORWARD, 0)
        self.left.spin(FORWARD, 0)

        # necessary local variables, totalError over the entire loop and i: iterations
        totalError:float = 0.0
        i = 0

        # calculates the initial error, paying mind to the smallest angles.
        if desiredValue - self.yourSensor() > 0:
            if desiredValue - self.yourSensor() <= 180:
                error:float = desiredValue - self.yourSensor()
            elif desiredValue - self.yourSensor() > 180:
                error:float = desiredValue - self.yourSensor() - 360
        else:
            if desiredValue - self.yourSensor() >= -180:
                error:float = desiredValue - self.yourSensor()
            elif desiredValue - self.yourSensor() < -180:
                error:float = 360 + (desiredValue - self.yourSensor())

        # adds initial error to the errorList used in detecting stabilization and previousError used in the derivative
        errorList = [error]
        previousError:float = error

        # runs the PID loop until setpoint is stabilized by detecting the absolute smallest error
        while abs(max(errorList, key=abs)) > tolerance:
            # counts iterations
            i += 1
            #calculates error, paying mind to smallest angles
            if desiredValue - self.yourSensor() > 0:
                if desiredValue - self.yourSensor() <= 180:
                    error:float = desiredValue - self.yourSensor()
                elif desiredValue - self.yourSensor() > 180:
                    error:float = desiredValue - self.yourSensor() - 360
            else:
                if desiredValue - self.yourSensor() >= -180:
                    error:float = desiredValue - self.yourSensor()
                elif desiredValue - self.yourSensor() < -180:
                    error:float = 360 + (desiredValue - self.yourSensor())

            # calculates derivative using error, previousError and sampling time
            derivative = (error - previousError) / 0.050
            # adds current error to totalError used in integral term
            totalError += error
            # sets output to the total PID equation or the speedCap
            self.output = min(error * self.KP + derivative * self.KD + (totalError * 0.050) * self.KI, (self.speedCap if error * self.KP + derivative * self.KD + (totalError * 0.050) * self.KI > 0 else -self.speedCap), key=abs)
            # sets totalError to zero if speedCap reached to prevent integral windup 
            if abs(self.output) == self.speedCap:
                totalError = 0
            # sets motor velocity to the PID output
            self.left.set_velocity(self.output, PERCENT)
            self.right.set_velocity(-self.output, PERCENT)
            # waits 50 MS to lighten program load
            wait(50)
            # sets previousError to current error for derivative term 
            previousError = error
            # adds error to errorList and deletes the first error if the list is longer than the settleTime divided by sampling time
            errorList.append(error)
            if len(errorList) > settleTime/0.050:
                errorList.pop(0)

            # save one row of PID data to buffer
            data_buffer += str(i * 0.050) + ","
            data_buffer += "%.3f" % (error*self.KP) + ","
            data_buffer += "%.3f" % (derivative * self.KD)  + ","
            data_buffer += "%.3f" % (totalError * 0.050 * self.KI) + ","
            data_buffer += "%.3f" % self.output + ","
            data_buffer += str(desiredValue) + ","
            data_buffer += "%.3f" % self.yourSensor() + "\n"

            # breaks the pid loop if the stopButton is pressed
            if stopButton and stop.isPressed(self.brain.screen.x_position(),self.brain.screen.y_position()):
                break
        
        # saves the data buffer onto the SD card as a SCV
        self.brain.sdcard.savefile(sd_file_name, bytearray(data_buffer, 'utf-8'))

# --------------------
# PID setup
# --------------------
# create a turnPID instance for drivetrain rotation
rotatePID = turnPID(yourSensor= gyro.heading , brain = brain, leftMotorGroup=left, rightMotorGroup=right, speedCap= 20,
                     KP = 0.34,
                     KI = 0.13,
                     KD = 0.014
                     )

# --------------------
# autonomous routines
# --------------------
def graph():
    """Autonomous routine to graph turn PID for various angles.
    """
    for i in range(30, 360, 30):
        right.spin(FORWARD, 0)
        left.spin(FORWARD, 0)
        rotatePID.graph(i, 2,sd_file_name='turnPID'+ str(i) + '.csv', stopButton=True)
        right.stop(HOLD)
        left.stop(HOLD)
        wait(2, SECONDS)
        right.spin(FORWARD, 0)
        left.spin(FORWARD, 0)
        rotatePID.graph(0, 2,sd_file_name='turnPID'+ str(-i) + '.csv', stopButton=True)
        wait(2, SECONDS)
        right.stop(HOLD)
        left.stop(HOLD)

# --------------------
# UI classes
# --------------------
class button:
    """touchscreen button object

    Parameters:
        height, width: size of rectangle
        posX, posY: top-left position on brain screen
        color: VEX Color
        text: label shown on the button
    """

    def __init__(self, height:int, width:int, posX:int, posY:int, color, text:str) -> None:
        self.height = height
        self.width = width
        self.posX = posX
        self.posY = posY
        self.Pressed = False
        self.color = color
        self.text = text

    def draw(self):
        """Draw the button on the brain screen."""
        brain.screen.set_pen_color(self.color)
        brain.screen.draw_rectangle(self.posX, self.posY, self.width, self.height, self.color)
        brain.screen.set_pen_color(Color.BLACK)
        brain.screen.print_at(self.text, x = self.posX + 10, y = self.posY + self.height//2 - 5, opaque = False)

    def isPressed(self, touchX:int, touchY:int) -> bool:
        """Return True if the provided touch coordinates are inside this button."""
        if touchX > self.posX and touchX < self.posX + self.width and touchY > self.posY and touchY < self.posY + self.height:
            self.Pressed = True
        else:
            self.Pressed = False
        return self.Pressed

# --------------------
# competition
# --------------------
    
def user_control():
    pass

# create competition instance
comp = Competition(user_control, graph)