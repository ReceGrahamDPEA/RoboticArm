# ////////////////////////////////////////////////////////////////
# //                     IMPORT STATEMENTS                      //
# ////////////////////////////////////////////////////////////////

import math
import sys
import time

from kivy.app import App
from kivy.lang import Builder
from kivy.core.window import Window
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.uix.button import Button
from kivy.uix.floatlayout import FloatLayout
from kivy.graphics import *
from kivy.uix.popup import Popup
from kivy.uix.label import Label
from kivy.uix.widget import Widget
from kivy.uix.slider import Slider
from kivy.uix.image import Image
from kivy.uix.behaviors import ButtonBehavior
from kivy.clock import Clock
from kivy.animation import Animation
from functools import partial
from kivy.config import Config
from kivy.core.window import Window
from pidev.kivy import DPEAButton
from pidev.kivy import PauseScreen
from time import sleep
import RPi.GPIO as GPIO 
from pidev.stepper import stepper
from pidev.Cyprus_Commands import Cyprus_Commands_RPi as cyprus


# ////////////////////////////////////////////////////////////////
# //                      GLOBAL VARIABLES                      //
# //                         CONSTANTS                          //
# ////////////////////////////////////////////////////////////////
START = True
STOP = False
UP = False
DOWN = True
ON = True
OFF = False
YELLOW = .180, 0.188, 0.980, 1 #Used for kivy colors
BLUE = 0.917, 0.796, 0.380, 1 #Used for kivy colors
CLOCKWISE = 0 #Don't think this does anything, but don't want to delete it without testing
COUNTERCLOCKWISE = 1 #Don't think this does anything, but don't want to delete it without testing
ARM_SLEEP = 2.5 #Don't think this does anything, but could be used in the auto in the place of 3 seconds.
DEBOUNCE = 0.10 #Don't think this does anything either

Magnet_On = 1 #Sets the magnet to on when used. Makes the magnet easier to use
Magnet_Off = .5 #Sets the magnet to off when used. Makes the magnet easier to use

lowerTowerPosition = 7300 #position of the lower tower
upperTowerPosition = 9495 #position of the upper tower

# ////////////////////////////////////////////////////////////////
# //            DECLARE APP CLASS AND SCREENMANAGER             //
# //                     LOAD KIVY FILE                         //
# ////////////////////////////////////////////////////////////////
class MyApp(App):

    def build(self):
        self.title = "Robotic Arm"
        return sm

Builder.load_file('main.kv')
Window.clearcolor = (.1, .1,.1, 1) # (WHITE)

cyprus.open_spi()

# ////////////////////////////////////////////////////////////////
# //                    SLUSH/HARDWARE SETUP                    //
# ////////////////////////////////////////////////////////////////

sm = ScreenManager()
arm = stepper(port=0, micro_steps=32, hold_current=20, run_current=20, accel_current=20, deaccel_current=20,
             steps_per_unit=200, speed=1)
cyprus.setup_servo(2)

# ////////////////////////////////////////////////////////////////
# //                       MAIN FUNCTIONS                       //
# //             SHOULD INTERACT DIRECTLY WITH HARDWARE         //
# ////////////////////////////////////////////////////////////////
	
class MainScreen(Screen):
    version = cyprus.read_firmware_version()
    armPosition = 0
    lastClick = time.clock()
    arm_state = 0
    magnet_state = 0

    ball_lower = 2
    ball_upper = 2


    def __init__(self, **kwargs):
        super(MainScreen, self).__init__(**kwargs)
        self.initialize()

        Clock.schedule_interval(self.whereIsTheBall, 1)

    def debounce(self):
        processInput = False
        currentTime = time.clock()
        if ((currentTime - self.lastClick) > DEBOUNCE):
            processInput = True
        self.lastClick = currentTime
        return processInput

    def toggleArm(self):

        """Toggles the arm going up or down whether "arm_state" is 0 or 1"""

        if self.arm_state == 0:

            cyprus.set_pwm_values(1, period_value=100000, compare_value=100000, compare_mode=cyprus.LESS_THAN_OR_EQUAL)
            print("arm going down?")
            self.arm_state += 1

        else:

            cyprus.set_pwm_values(1, period_value=100000, compare_value=0, compare_mode=cyprus.LESS_THAN_OR_EQUAL)
            print("arm going up?")
            self.arm_state -= 1

    def toggleMagnet(self):

        """Toggles the magnet based on whether "magnet_state" is 1 or 0"""

        if self.magnet_state == 0:

            cyprus.set_servo_position(2, Magnet_On)
            print("Magnet On")
            self.magnet_state += 1

        else:

            cyprus.set_servo_position(2, Magnet_Off)
            print("Magnet Off")
            self.magnet_state -= 1
        
    def auto(self):

        """Calls the auto ball function and controls which tower the ball goes to depending on the information the
        sensors are getting. """

        if self.ball_lower == 1:

            self.auto_ball(lowerTowerPosition, upperTowerPosition)

        if self.ball_upper == 1:

            self.auto_ball(upperTowerPosition, lowerTowerPosition)

    def auto_ball(self, tower1, tower2):

        """The big function. Automatically sends the arm to get the ball and will go to whichever tower the ball is
        on. This function is dependent on the above function "auto" to fill in the values for "tower1" tower2" """

        arm.goTo(tower1)
        arm.wait_move_finish()
        sleep(.5)
        cyprus.set_pwm_values(1, period_value=100000, compare_value=100000, compare_mode=cyprus.LESS_THAN_OR_EQUAL)
        sleep(.5)
        cyprus.set_servo_position(2, Magnet_On)
        sleep(.5)
        cyprus.set_pwm_values(1, period_value=100000, compare_value=0, compare_mode=cyprus.LESS_THAN_OR_EQUAL)
        sleep(2)

        arm.goTo(tower2)
        sleep(3)
        cyprus.set_pwm_values(1, period_value=100000, compare_value=100000, compare_mode=cyprus.LESS_THAN_OR_EQUAL)
        sleep(.5)
        cyprus.set_servo_position(2, Magnet_Off)
        sleep(.5)
        cyprus.set_pwm_values(1, period_value=100000, compare_value=0, compare_mode=cyprus.LESS_THAN_OR_EQUAL)
        print("Done!")

    def setArmPosition(self):

        """Interacts with the slider and updates the position of the arm. Also prints the value of the moveArm slider
        from the Kivy file """

        if not self.ids.moveArm.value == 0:
            print(self.ids.moveArm.value)
            arm.goTo(int(self.ids.moveArm.value))

        else:
            print(self.ids.moveArm.value)
            arm.goHome()

    def homeArm(self):

        """I'm not sure if ths actually does anything, but it's supposed to home the arm."""

        arm.home(self.homeDirection)
        
    def whereIsTheBall(self, dt):

        """Creating variables with only one clock function. This should take less processing power than running
        a million clocks"""

        if cyprus.read_gpio() & 0b0010:  # binary bitwise AND of the value returned from read.gpio()
            self.ball_lower = 0
            print("lower_state " + str(self.ball_lower))

        else:
            self.ball_lower = 1
            print("lower_state " + str(self.ball_lower))

        if cyprus.read_gpio() & 0b0001:
            self.ball_upper = 0
            print("upper_state " + str(self.ball_upper))

        else:
            self.ball_upper = 1
            print("upper_state " + str(self.ball_upper))
        
    def initialize(self):

        """Function that is run when the program is initialized. Is called by init of the MainScreen class"""

        cyprus.set_pwm_values(1, period_value=100000, compare_value=0, compare_mode=cyprus.LESS_THAN_OR_EQUAL)
        cyprus.set_servo_position(2, Magnet_Off)
        arm.start_relative_move(-2)
        arm.wait_move_finish()
        arm.set_as_home()
        print("Home arm and turn off magnet")

    def resetColors(self):

        """Functionality for kivy, sets different color for different widgets. Variables can be found near the top of
        this file """

        self.ids.armControl.color = YELLOW
        self.ids.magnetControl.color = YELLOW
        self.ids.auto.color = BLUE

    def quit(self):

        """Quit function. This should set everything to "default" (That being with the arm up, the magnet off,
        and arm itself freed """

        arm.free_all()
        sleep(.5)
        cyprus.set_pwm_values(1, period_value=100000, compare_value=0, compare_mode=cyprus.LESS_THAN_OR_EQUAL)
        cyprus.set_servo_position(2, Magnet_Off)
        cyprus.close()
        GPIO.cleanup()
        print("Exit")
        MyApp().stop()
    
sm.add_widget(MainScreen(name = 'main'))


# ////////////////////////////////////////////////////////////////
# //                          RUN APP                           //
# ////////////////////////////////////////////////////////////////

MyApp().run()
cyprus.close_spi()
