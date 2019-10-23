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

  private ElapsedTime runtime = new ElapsedTime();

  @Override
  public void runOpMode() {
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    driveChassis = new MecanumDriveChassis(hardwareMap);

    // set deadzone to minimize unwanted stick input.
    gamepad1.setJoystickDeadzone((float)0.05);

    // Wait for the game to start (driver presses PLAY)
    waitForStart();
    runtime.reset();

    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {

      // send joystick inputs to the drive chassis
      driveChassis.drive(gamepad1.left_stick_y, gamepad1.left_stick_x,
                         gamepad1.right_stick_x, telemetry);

      // Show the elapsed game time and wheel power.
//      telemetry.addData("Status", "Run Time: " + runtime.toString());
//      telemetry.addData("Stick", "Y_left (%.2f), X_left (%.2f)",
//          gamepad1.left_stick_y, gamepad1.left_stick_x);
//      telemetry.update();
    }
  }
}
