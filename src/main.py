# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       ruben                                                        #
# 	Created:      11/17/2025, 3:59:26 PM                                       #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *
from PID import *

# cevice configurations
brain=Brain()
gyro = Inertial(Ports.PORT1)
motor = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)

controller = PID(
    yourSensor= gyro,
    brain= brain,
    func= motor.spin,
    KP=1,
    KI=0,
    KD=0
)

def changeConstants(new_KP: float, new_KI: float, new_KD: float):
    controller.KP = new_KP
    controller.KI = new_KI
    controller.KD = new_KD

controller.tune(desiredValue=90, tollerance=1, sd_file_name="pidData.csv", stopButton=True)
        
