import wpilib
import commands2.button

from toolkit.oi import (
    XBoxController,
    LogitechController,
    JoystickButton,
    DefaultButton
)

controllerDRIVER = XBoxController
controllerOPERATOR = XboxController

class Controllers:
    DRIVER: int = 0
    OPERATOR: int = 1

    DRIVER_CONTROLLER = wpilib.Joystick(0)
    OPERATOR_CONTROLLER = wpilib.Joystick(1)

class KeyMap:
    class Drivetrain:
        pass
    
    class Intake:
        pass

    class Shooter:
        pass

    class Climber:
        pass

    class Indexer:
        
        SHOOT_FUEL = commands2.button.Trigger(
            lambda: Controllers.DRIVER_CONTROLLER.getRawAxis(-controllerDRIVER.RT) > constants.trigger_threshold 
        )
        
        FORCE_INDEX = commands2.button.Commands(
            Joysticks.joysticks[Controllers.OPERATOR], ControllerOPERATOR.A
        )

        FORCE_INDEX = commands2.button.Commands(
            Joysticks.joysticks[Controllers.DRIVER], ControllerDRIVER.A
        )

        REVERSE_INDEX = commands2.button.Commands(
            Joysticks.joysticks[Controllers.OPERATOR], ControllerOPERATOR.Y
        )

        #PASS?
    
         
