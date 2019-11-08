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

  private final boolean suckStones = true;
  private final boolean spitStones = false;

  private ElapsedTime runtime = new ElapsedTime();
  // a timer for the various automation activities.
  private ElapsedTime eventTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

  // the time to hold before allowing state change on button.
  private final double stoneSpitRunTime = 2.0;
  private double stoneSpitTimeoutTime = 0;
  private boolean stoneSpitTimerRunning = false;

  @Override
  public void runOpMode() {
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    driveChassis = new MecanumDriveChassis(hardwareMap);
    manipulatorPlatform = new ManipulatorPlatform(hardwareMap);
    leds = new LEDs(hardwareMap);
    leds.goOff();

    // set dead zone to minimize unwanted stick input.
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

      // *** Driver controls (game pad 1)

      // provide a throttle capability to run the bot at one of two speeds.
      if (gamepad1.right_trigger > 0.5 && !goingFast )  // Go fast
      {
        driveChassis.turboMode(highSpeed);
        goingFast = true;
      }
      else if (gamepad1.right_trigger < 0.5 && goingFast)
      {
        driveChassis.turboMode(lowSpeed);
        goingFast = false;
      }

      // Closes the latches to drag the building platform.
      if (gamepad1.left_bumper) // Latch
      {
        manipulatorPlatform.latchPosition(true);
      }

      // Open the latches that drag the building platform
      if (gamepad1.left_trigger > 0.5)  // Unlatch
      {
        manipulatorPlatform.latchPosition(false);
      }


      // Set the gatherer motors to collect.  Lowers the gather deck to the down position.
      // Looks for the stone to hit the limit switch then stops gathering and raises the
      // platform.
      if (gamepad1.a)
      {
        manipulatorPlatform.gatherOn(suckStones);
        manipulatorPlatform.gatherDown();
      }


      // Spit the gathered stones out
      // runs the motors in reverse for some set time then stops the motors.
      if (gamepad1.b && !stoneSpitTimerRunning)
      {
        manipulatorPlatform.gatherOn(spitStones);
        stoneSpitTimeoutTime = eventTimer.time() + stoneSpitRunTime;
        stoneSpitTimerRunning = true;
      }
      // cancel the stone spiting...
      if(stoneSpitTimerRunning && eventTimeOut(stoneSpitTimeoutTime))
      {
        manipulatorPlatform.gatherOff();
      }


      if (gamepad1.x)
      {
        manipulatorPlatform.gatherAbort();
      }

      if (gamepad1.y)
      {
        manipulatorPlatform.gatherOff();
      }


      // Second seat...  controls (game pad 2)
      if (gamepad2.left_bumper)  // grab
      {
        manipulatorPlatform.grab(false);
      }
      if (gamepad2.left_trigger > 0.5)  // release
      {
        // send joystick inputs to the bench
        manipulatorPlatform.grab(true);
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


      if (gamepad2.b)   // Extend
      {
        manipulatorPlatform.elevatorPosition(200);
      }

      if (gamepad2.a)   // retract
      {
        manipulatorPlatform.elevatorPosition(150);
      }


      if (gamepad2.x)
      {
        manipulatorPlatform.elevatorPosition(50);
      }

      if (gamepad2.y)
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

  // test the event time
  private boolean eventTimeOut ( double eventTime){
    return eventTimer.time() > eventTime;
  }
}
