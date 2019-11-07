package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Primary Tele-Op", group="Linear Opmode")
//@Disabled
public class TeleOpPrimary extends LinearOpMode {

  // Declare OpMode members.
  private MecanumDriveChassis driveChassis;
  private ManipulatorPlatform manipulatorPlatform;
  private LEDs leds;

  private final double highSpeed = 1.0;
  private final double lowSpeed = 0.5;


  private ElapsedTime runtime = new ElapsedTime();

  @Override
  public void runOpMode() {
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    driveChassis = new MecanumDriveChassis(hardwareMap);
    manipulatorPlatform = new ManipulatorPlatform(hardwareMap);
    leds = new LEDs(hardwareMap);
    leds.goOff();

    // set deadzone to minimize unwanted stick input.
    gamepad1.setJoystickDeadzone((float)0.05);

    boolean goingFast = false;

    // Wait for the game to start (driver presses PLAY)
    waitForStart();
    runtime.reset();

    leds.goConfetti();

    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {

      // send joystick inputs to the drive chassis
      driveChassis.drive(gamepad1.left_stick_y, gamepad1.left_stick_x,
                         gamepad1.right_stick_x, telemetry);

    // Driver controls (game pad 1)
      if (gamepad1.right_trigger > 0.5 && !goingFast )  // Go fast
      {
        MecanumDriveChassis.turboMode(highSpeed);
        goingFast = true;
      }
      else if (gamepad1.right_trigger < 0.5 && goingFast)
      {
        MecanumDriveChassis.turboMode(lowSpeed);
        goingFast = false;
      }

      if (gamepad1.x)
      {
        manipulatorPlatform.gatherOn();
      }

      if (gamepad1.y)
      {
        manipulatorPlatform.gatherOff();
      }


      if (gamepad1.a) // Latch
      {
        manipulatorPlatform.latchPosition(true);
      }

      if (gamepad1.b)  // Unlatch
      {
        manipulatorPlatform.latchPosition(false);
      }


      // Second seat...  controls (game pad 2)
      if (gamepad1.left_bumper)  // grab
      {
        manipulatorPlatform.grab(true);
      }
      if (gamepad1.left_trigger > 0.5)  // release
      {
        // send joystick inputs to the bench
        manipulatorPlatform.grab(false);
      }


      if (gamepad2.right_trigger > 0.5)  // 90 degrees
      {
        manipulatorPlatform.rotate(true);
      }
      if (gamepad2.left_trigger > 0.5)  // 0 degrees
      {
        // send joystick inputs to the bench
        manipulatorPlatform.grab(false);
      }


      if (gamepad2.x)   // Extend
      {
        manipulatorPlatform.extenderPosition(200);
      }

      if (gamepad2.b)   // retract
      {
        manipulatorPlatform.extenderPosition(0);
      }


      if (gamepad2.y)
      {
        manipulatorPlatform.elevatorPosition(200);
      }

      if (gamepad2.a)
      {
        manipulatorPlatform.elevatorPosition(0);
      }


      // Show the elapsed game time and wheel power.
//      telemetry.addData("Status", "Run Time: " + runtime.toString());
//      telemetry.addData("Stick", "Y_left (%.2f), X_left (%.2f)",
//          gamepad1.left_stick_y, gamepad1.left_stick_x);
//      telemetry.update();
    }
  }
}
