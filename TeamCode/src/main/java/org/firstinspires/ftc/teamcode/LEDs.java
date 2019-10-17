package org.firstinspires.ftc.teamcode;

public class LEDs {

  Servo blinkin;
  private final double TURN_HOTPINK = 0.57;
  private final double TURN_WHITE = -0.81;
  private final double TURN_BLUE = 0.87;
  private final double TURN_BLACK = 0.99;
  private final double TURN_FIRE = -0.57;
    // Initialize Blinkin module.

  public void LEDs {
    blinkin = hardwareMap.servo.get("blinkin");
    loop () {
      blinkBlackandBlue();
    }
  }



    // Set Blinkin pattern.

  public void blinkBlackandBlue() {
    // turn light blue
    blinkin.setPosition(TURN_BLACK);
    //wait 0.5 seconds.
    pause(500);
    // Turn light white
    blinkin.setPosition(TURN_BLUE);
    // wait 0.5 seconds
    pause(500);
  }
  public void blinkpinkandwhite() {
    // turn light blue
    blinkin.setPosition(TURN_WHITE);
    //wait 0.5 seconds.
    pause(500);
    // Turn light white
    blinkin.setPosition(TURN_PINK);
    // wait 0.5 seconds
    pause(500);
  }
  public void blink (double color,int timer) {
    blinkin.setPosition(color);
    // wait x seconds
    pause(timer);

    public void blink ( double TURN_WHITE, int pause ) {
      blinkin.setPosition(-0.81);
      // wait so and so seconds
      pause (500);

    }
  }
}
