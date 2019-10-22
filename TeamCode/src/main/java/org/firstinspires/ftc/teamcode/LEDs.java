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


  public void goBlue()
  {
    pattern = LIGHT_CHASE_BLUE;
    blinkin.setPattern(pattern);
  }


  public void goRed()
  {
    pattern = LIGHT_CHASE_RED;
    blinkin.setPattern(pattern);
  }
}
