from wpilib import AddressableLED, LEDPattern, Color
import ntcore
import constants
from wpilib import DriverStation
from commands2 import Subsystem

class LEDs(Subsystem):

    def __init__(self):

        self.led = AddressableLED(constants.leds_id)
        self.led.setLength(constants.led_length)
        self.led_buffer = [self.led.LEDData() for i in range(constants.led_length)]

        self.pattern = LEDPattern.solid(Color.kBlack)

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

    def update_leds(self):
        """
        updates LED pattern
        """
        self.pattern.applyTo(self.led_buffer)
        self.led.setData(self.led_buffer)

    def set_brightness(self, brightness):
        """
        sets LED brightness
        """
        self.pattern.atBrightness(brightness)
        self.update_leds()

    def set_solid(self, r: int, g: int, b: int):
        """
        sets LEDs as a solid color
        """
        self.pattern = LEDPattern.solid(Color(r/255, g/255, b/255))
        self.update_leds()

    def set_rainbow(self):
        """
        sets LEDs to a rainbow pattern
        """
        self.pattern = LEDPattern.rainbow(constants.rainbow_saturation, constants.rainbow_value)
        self.update_leds()

    def set_gradient(self, r1: int, g1: int, b1: int, r2: int, g2: int, b2: int):
        """
        sets LEDs to a (continuous) gradient pattern
        """
        self.pattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color(r1/255, g1/255, b1/255), Color(r2/255, g2/255, b2/255))
        self.update_leds()

    def set_progress_mask(self, current_value, max_value, r: int, g: int, b: int):
        """
        sets progress bar of any color of choice pattern for LEDs 
        """
        self.pattern = LEDPattern.progressMaskLayer(current_value / max_value).solid(Color(r/255, g/255, b/255))
        self.update_leds()
    
    def reverse_pattern(self):
        """
        reverses direction of given pattern
        """
        self.pattern.reversed()
        self.update_leds()
    
    def set_scroll(self, r1: int, g1: int, b1: int, r2: int, g2: int, b2: int):
        """
        sets a scrolling pattern for the whole LED strips
        """
        self.set_gradient(r1, g1, b1, r2, g2, b2)
        self.pattern.scrollAtRelativeSpeed(constants.scroll_speed)
        self.update_leds()

    def set_blink(self, r1: int, g1: int, b1: int, r2: int, g2: int, b2: int):
        """
        set blink pattern for the LEDs
        """
        self.set_gradient(r1, g1, b1, r2, g2, b2)
        self.pattern.blink(constants.blink_time)
        self.update_leds()

    def set_alternate(self, r1: int, g1: int, b1: int, r2: int, g2: int, b2: int):
        """
        LED strip alternates between two colors
        """
        self.alternate = []
        for i in range(constants.led_length):
            if i % 2 == 0:
                self.alternate.append(
                    (i / constants.led_length, Color(r1 / 255, g1 / 255, b1 / 255))
                )
            elif i % 2 == 1:
                self.alternate.append(
                    (i / constants.led_length, Color(r2 / 255, g2 / 255, b2 / 255))
                )
        self.pattern = LEDPattern.steps(self.alternate)
        self.update_leds()
    
    