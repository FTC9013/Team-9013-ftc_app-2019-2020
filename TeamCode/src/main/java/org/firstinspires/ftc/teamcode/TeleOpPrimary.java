package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;



/*
Motor Names for Configuration

Hub 2:
0 = eMotor (rev robotics 40:1 HD Hex)
1 = aMotor (rev robotics 40:1 HD Hex)
0 = cServo

Hub 3:

0 = lFront (NeveRest 40 Gearmotor)
1 = rFront (NeveRest 40 Gearmotor)
2 = lRear (NeveRest 40 Gearmotor)
3 = rRear (NeveRest 40 Gearmotor)
*/

@TeleOp(name="Primary Tele-Op", group="Linear Opmode")
//@Disabled
public class TeleOpPrimary extends LinearOpMode {

  // Declare OpMode members.
  private MecanumDriveChassis driveChassis;
  private ManipulatorPlatform manipulatorPlatform;
  private LEDs leds;

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

      if (gamepad1.x)
      {
        manipulatorPlatform.gatherOn();
      }

      if (gamepad1.y)
      {
        manipulatorPlatform.gatherOff();
      }


      if (gamepad1.a)
      {
        manipulatorPlatform.latchPosition(true);
      }

      if (gamepad1.b)
      {
        manipulatorPlatform.latchPosition(false);
      }

      // Second seat...  controls (game pad 2)

      if (gamepad2.right_bumper)  // grab
      {
        manipulatorPlatform.grab(true);
      }
      if (gamepad2.left_bumper)  // release
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

      if (gamepad1.b)   // retract
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
