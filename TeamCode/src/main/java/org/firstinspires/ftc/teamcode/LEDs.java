package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;



import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.*;

public class LEDs
{

  private RevBlinkinLedDriver blinkin;
  private int currnetPpattern = 0;

  // list of led patterns to sequence through.  add / remove as you like.
  private RevBlinkinLedDriver.BlinkinPattern ledSequence[] =
      {
          TWINKLES_LAVA_PALETTE,
          SHOT_BLUE,
          FIRE_LARGE,
          COLOR_WAVES_OCEAN_PALETTE,
          FIRE_MEDIUM,
          RAINBOW_WITH_GLITTER

          // edit this list to add more options.

      };

  LEDs(HardwareMap hardwareMap)
  {
    // Initialize the hardware variables. Note that the strings used here as parameters
    // to 'get' must correspond to the names assigned during the robot configuration
    // step (using the FTC Robot Controller app on the phone).

    // Get blinkin LED controller (emulates a servo...)
    blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "leds");
    currnetPpattern = 0;
  }

  public void goOff()
  {
    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
  }

  public void goStrobe()
  {
    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER);
  }

  public void goConfetti()
  {
    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
  }

  public void goOcean()
  {
    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_OCEAN_PALETTE);
  }
  public void goFireLarge()
  {
    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE);
  }


  public void goChangeColor()
  {
    // go to the next index, check to make sure it's a legal value.
    ++currnetPpattern;
    if( currnetPpattern < ledSequence.length && currnetPpattern >= 0 )
    {
      blinkin.setPattern(ledSequence[currnetPpattern]);
    }
    // wrap if currentPattern is at the end of the pattern list
    else if ( currnetPpattern >= ledSequence.length)
    {
      currnetPpattern = 0;
      blinkin.setPattern(ledSequence[currnetPpattern]);
    }
  }


   public void goChangeColor(int colorL)
   {
     // test to make sure the index isn't too big or negative.
     if(colorL < ledSequence.length && !(colorL < 0) )
     {
       blinkin.setPattern(ledSequence[colorL]);
     }
   }

}



