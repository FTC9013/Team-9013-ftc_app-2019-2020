package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;



import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.*;

public class LEDs {

  private RevBlinkinLedDriver blinkin;
  private RevBlinkinLedDriver.BlinkinPattern pattern;


  LEDs(HardwareMap hardwareMap)
  {
    // Initialize the hardware variables. Note that the strings used here as parameters
    // to 'get' must correspond to the names assigned during the robot configuration
    // step (using the FTC Robot Controller app on the phone).

    // Get blinkin LED controller (emulates a servo...)
    blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "leds");
  }


  public void goStrobe()
  {
    pattern = RAINBOW_WITH_GLITTER;
    blinkin.setPattern(pattern);
  }

  public void goConfetti()
  {
    pattern = CONFETTI;
    blinkin.setPattern(pattern);
  }

  public void goOcean()
  {
    pattern = SINELON_OCEAN_PALETTE;
    blinkin.setPattern(pattern);
  }
  public void goFireLarge()
  {
    pattern = FIRE_LARGE;
    blinkin.setPattern(pattern);
  }
}
