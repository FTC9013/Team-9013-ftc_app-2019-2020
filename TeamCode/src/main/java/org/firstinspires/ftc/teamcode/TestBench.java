package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.logging.Level;

@TeleOp(name="TestBench", group="Linear Opmode")
//@Disabled
public class TestBench extends LinearOpMode {

  // Declare OpMode members.
  private TestBenchPlatform testBench;
  private LEDs leds;

  private ElapsedTime runtime = new ElapsedTime();

  @Override
  public void runOpMode() {
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    testBench = new TestBenchPlatform(hardwareMap);
    leds = new LEDs(hardwareMap);

    // Wait for the game to start (driver presses PLAY)
    waitForStart();
    runtime.reset();

    // run until the end of the match (driver presses STOP)
    while (opModeIsActive())
    {
      // simple button press detection to call actuator test methods
      // Some of this may be reused for TeleOp, but will likely require timer support for debounce
      // of button presses.



      if (gamepad1.x) {
        // send joystick inputs to the bench
        testBench.elevatorPosition(100);
      }
      if (gamepad1.y) {
        // send joystick inputs to the bench
        testBench.elevatorPosition(0);
      }

      if (gamepad1.right_bumper) {
        // send joystick inputs to the bench
        testBench.latchPosition(1);
      }
      if (gamepad1.left_bumper) {
        // send joystick inputs to the bench
        testBench.latchPosition(0);
      }


      if (gamepad1.a) {
        // send joystick inputs to the leds
        leds.goRed();
      }

      if (gamepad1.b) {
        // send joystick inputs to the leds
        leds.goBlue();
      }







    }
  }
}
