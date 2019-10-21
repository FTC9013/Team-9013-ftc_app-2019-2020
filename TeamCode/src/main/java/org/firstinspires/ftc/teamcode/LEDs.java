package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;


import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.*;

public class LEDs {

  private final double TURN_HOTPINK = 0.57;
  private final double TURN_WHITE = -0.81;
  private final double TURN_BLUE = 0.87;
  private final double TURN_BLACK = 0.99;
  private final double TURN_FIRE = -0.57;
  
  private RevBlinkinLedDriver blinkin;
  private RevBlinkinLedDriver.BlinkinPattern pattern;

  LEDs(RevBlinkinLedDriver leds)
  {
    this.blinkin = leds;
    blinkpinkandwhite();
  }



    // Set Blinkin pattern.

//  public void blinkBlackandBlue() {
//    // turn light blue
//    blinkin.setPattern(TURN_BLACK);
//    //wait 0.5 seconds.
//    //pause(500);
//    // Turn light white
//    blinkin.setPosition(TURN_BLUE);
//    // wait 0.5 seconds
//    //pause(500);
//  }
  
  
  public void blinkpinkandwhite()
  {
    pattern = LIGHT_CHASE_BLUE;
    // turn light blue
    blinkin.setPattern(pattern);

  }
//  public void blink (double color,int timer) {
//    blinkin.setPosition(color);
//    // wait x seconds
   // pause(timer);

 //   public void blink ( double TURN_WHITE, int pause ) {
   //   blinkin.setPosition(-0.81);
      // wait so and so seconds
      //pause (500);

//    }
//  }
}
