from wpilib import AddressableLED
import ntcore
import constants
from commands2 import Subsystem

class LEDs(Subsystem):

    def __init__(self):

        self.led = AddressableLED(constants.leds_id)
        self.led 
        #self.led_buffer = 
    def enable_leds(self):
        """
        enables the LEDs
        """
        self.led.start()

    def disable_leds(self):
        """
        disables the LEDs
        """
        self.led.stop()
