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

  private final int extenderRetracted  = 0;
  private final int extenderExtended  = 750;

  private final int elevatorDown  = 0;
  private final int elevatorLevelOne    = 100;
  private final int elevatorLevelTwo    = 200;
  private final int elevatorLevelThree  = 400;

  private final boolean suckStones = true;
  private final boolean spitStones = false;

  private boolean suckingStones = false;
  private boolean spittingStones = false;


  private ElapsedTime runtime = new ElapsedTime();
  // a timer for the various automation activities.
  private ElapsedTime eventTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

  // the time to hold before allowing state change on button.
  private final double stoneSpitRunTime = 2.5;
  private double stoneSpitTimeoutTime = 0;  // stores the time the event should end
  private boolean stoneSpitTimerRunning = false;

  // the time to hold to allow stone to settle flat then close holder.
  private final double stoneSettleDelayTime = 0.5;
  private double stoneSettleDelayTimeoutTime = 0;  // stores the time the event should end
  private boolean stoneSettleDelayTimerRunning = false;
  private final double stoneSettleTime = 0.25;
  private double stoneSettleTimeoutTime = 0;  // stores the time the event should end
  private boolean stoneSettleTimerRunning = false;

  private final double colorSpeed = 10; // change colors every 10 seconds
  private double LEDTimeoutTime = 0;

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

    //leds.goConfetti();  // start the LEDs in confetti

    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {

      //set up counter

      // steps through color sequences every 'colorSpeed' seconds.
      if ( eventTimeOut( LEDTimeoutTime ) )
      {
        LEDTimeoutTime = eventTimer.time() + colorSpeed;  // load for next cycle
      //  leds.goChangeColor();
      }

      // send joystick inputs to the drive chassis
      driveChassis.drive(gamepad1.left_stick_y, gamepad1.left_stick_x,
                         gamepad1.right_stick_x, telemetry);

      // *** Driver controls (game pad 1)

      // provide a throttle capability to run the bot at one of two speeds.
      if (gamepad1.right_trigger > 0.5 && goingFast )  // Go fast
      {
        driveChassis.setSpeedScale(lowSpeed);
        goingFast = false;
      }
      else if (gamepad1.right_trigger < 0.5 && !goingFast)
      {
        driveChassis.setSpeedScale(highSpeed);
        goingFast = true;
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
      if (gamepad1.a && !suckingStones && !spittingStones)
      {
        suckingStones = true;
        manipulatorPlatform.elevatorPosition(elevatorLevelOne);
        manipulatorPlatform.extenderPosition(extenderRetracted);
        manipulatorPlatform.gatherOn(suckStones);
        manipulatorPlatform.gatherDown();
        manipulatorPlatform.gatherHold();
      }

      if (suckingStones && manipulatorPlatform.stonePresent())
      {
        manipulatorPlatform.gatherOff();
        manipulatorPlatform.gatherUp();
        manipulatorPlatform.grabberRelease();
        manipulatorPlatform.elevatorPosition(elevatorDown);
        stoneSettleDelayTimerRunning = true;
        stoneSettleDelayTimeoutTime = eventTimer.time() + stoneSettleDelayTime;
        suckingStones = false;
      }

      // wait for the gather to tip back up before settling the stone.
      if(stoneSettleDelayTimerRunning && eventTimeOut(stoneSettleDelayTimeoutTime))
      {
        manipulatorPlatform.gatherRelease();
        stoneSettleTimeoutTime = eventTimer.time() + stoneSettleTime;
        stoneSettleTimerRunning = true;
        stoneSettleDelayTimerRunning = false;
      }

      // let the stone settle in the holder after gather.
      if(stoneSettleTimerRunning && eventTimeOut(stoneSettleTimeoutTime))
      {
        manipulatorPlatform.gatherHold();
        stoneSettleTimerRunning = false;
      }


      if (gamepad1.y)
      {
        manipulatorPlatform.gatherRelease();
      }

      // Spit the gathered stones out
      // runs the motors in reverse for some set time then stops the motors.
      if (gamepad1.b && !stoneSpitTimerRunning && !spittingStones)
      {
        suckingStones = false; // cancel any stone sucking...
        spittingStones = true;
        manipulatorPlatform.gatherOn(spitStones);
        stoneSpitTimeoutTime = eventTimer.time() + stoneSpitRunTime;
        stoneSpitTimerRunning = true;
      }

      // cancel the stone spiting...
      if(spittingStones && eventTimeOut(stoneSpitTimeoutTime))
      {
        manipulatorPlatform.gatherAbort(); // motors off and table up
        stoneSpitTimerRunning = false;
        spittingStones = false;
      }

      // abort gathering or spitting
      if (gamepad1.x)
      {
        manipulatorPlatform.gatherAbort(); // motors off and table up
        suckingStones = false;
        spittingStones = false;
        stoneSpitTimerRunning = false;
      }


      // ***************************
      // Second seat...  controls (game pad 2)

      //RIGHT JOY STICK
      // Forward push command Extender all the way out (to encoder count = XXXX)
      // Backward push command Extender all the way in (encoder count = 0)
      if (gamepad2.right_stick_y < -0.5) // forward in -Y
      {
        manipulatorPlatform.extenderPosition(extenderExtended);
      }

      if (gamepad2.right_stick_y > 0.5) // Backward is +Y
      {
        manipulatorPlatform.extenderPosition(extenderRetracted);
      }

      // D‐PAD – Controls Grabber head orientation (0° or 90°)
      // a. D‐Up commands Servo4 (Hub 2, servo port 4) to 0° (0)
      // b. D‐Down commands Servo4 to 90° (1)
      if (gamepad2.dpad_up)
      {
        manipulatorPlatform.rotate0();
      }

      if (gamepad2.dpad_down)
      {
        manipulatorPlatform.rotate90();
      }

      // LEFT BUMP – commands Grabber Servo3 (Hub 2, servo port 2) to close (0),
      //   and,Release Stone (Hub 2, Servo5, Port 5) to open.
      if (gamepad2.left_bumper)
      {
        manipulatorPlatform.grabberRelease();
        manipulatorPlatform.gatherHold();
      }

      // RIGHT BUMP – commands Servo3 (Hub 2, servo port 2) to open (1)
      if (gamepad2.right_bumper)  // grab
      {
        manipulatorPlatform.grabberGrab();
        manipulatorPlatform.gatherRelease();
      }

      // A – Commands Elevator motor (Hub 2, motor port 2) to Level 1 (encoder count X)
      if (gamepad2.a)   // retract
      {
        manipulatorPlatform.elevatorPosition(elevatorLevelOne);
      }

      // X – Commands Elevator motor (Hub 2, motor port 2) to Level 2 (encoder count Y)
      if (gamepad2.x)
      {
        manipulatorPlatform.elevatorPosition(elevatorLevelTwo);
      }

      // Y – Commands Elevator motor (Hub 2, motor port 2) to Level 3 (encoder count Z)
      if (gamepad2.y)
      {
        manipulatorPlatform.elevatorPosition(elevatorLevelThree);
      }

      // B – Commands Elevator motor (Hub 2, motor port 2) to Down/Home (encoder count 0)
      if (gamepad2.b)   // Extend
      {
        manipulatorPlatform.elevatorPosition(elevatorDown);
      }
    }
  }

  // test the event time
  private boolean eventTimeOut ( double eventTime)
  {
    return eventTimer.time() > eventTime;
  }
}
