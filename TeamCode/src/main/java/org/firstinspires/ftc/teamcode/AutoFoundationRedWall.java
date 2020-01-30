package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.LinkedList;
import java.util.Queue;

@Autonomous(name = "AutoFoundationRedWall", group = "Linear Opmode")

//@Disabled
public class AutoFoundationRedWall extends LinearOpMode
{

  // Declare OpMode members.
  private MecanumDriveChassisAutonomous driveChassis;
  private ManipulatorPlatform manipulatorPlatform;
  private LEDs leds;

  private ElapsedTime manipulateimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

  // save for later
  // private static final float mmPerInch = 25.4f;
  // private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
  // private static final float mmTargetHeight = (6) * mmPerInch; // the height of the center of the target image above the floor

  private boolean extendedFlag = false;
  
  private final int extenderRetracted  = 0;
  private final int extenderExtended  = 850;

  @Override
  public void runOpMode()
  {

    driveChassis = new MecanumDriveChassisAutonomous(hardwareMap);
    manipulatorPlatform = new ManipulatorPlatform(hardwareMap);
    leds = new LEDs(hardwareMap);
    leds.goOff();
  
    // build all the drive plans for drive by distance (time in seconds)
    //
    // Each leg of the trip is added to the queue in this code block.
    // As the opmode runs, the queue sent to the drive base for execution.
    //
    // mode:     {FORWARD, BACKWARDS, LEFT (strafe), RIGHT (strafe), TURN}
    // speed:    Ignored in TURN mode: the drive speed from 0-100%  (Slower speeds for longer
    //           times will be more precise)
    // angle:    Ignored in all but TURN mode:
    //           the desired angle of travel relative to the ZERO orientation in DEGREES
    //           ZERO is where the bot was facing when the IMU calibrated.
    //           desired angle in degrees 0 to 360 CCW
    // distance: Only used for FORWARD, BACKWARD, LEFT, RIGHT, modes:  the TIME in seconds to run
    //           the motors.

    Queue<Leg> TestAllFunctions = new LinkedList<>();
    TestAllFunctions.add(new Leg(Leg.Mode.LEFT, 50, 0, 1));
    TestAllFunctions.add(new Leg(Leg.Mode.RIGHT, 50, 0, 1));
    TestAllFunctions.add(new Leg(Leg.Mode.FORWARD, 50, 0, 1));
    TestAllFunctions.add(new Leg(Leg.Mode.TURN, 50, 90, 0));
    TestAllFunctions.add(new Leg(Leg.Mode.FORWARD, 50, 0, 1));
    TestAllFunctions.add(new Leg(Leg.Mode.TURN, 50, 180, 0));
    TestAllFunctions.add(new Leg(Leg.Mode.FORWARD, 50, 0, 1));
    TestAllFunctions.add(new Leg(Leg.Mode.TURN, 50, 270, 0));
    TestAllFunctions.add(new Leg(Leg.Mode.FORWARD, 50, 0, 1));
    TestAllFunctions.add(new Leg(Leg.Mode.TURN, 50, 0, 0));
    TestAllFunctions.add(new Leg(Leg.Mode.LEFT, 50, 0, 1));
    TestAllFunctions.add(new Leg(Leg.Mode.RIGHT, 50, 0, 1));


    Queue<Leg> LatchPt1 = new LinkedList<>();
    LatchPt1.add(new Leg(Leg.Mode.RIGHT, 50, 0, .7));
    LatchPt1.add(new Leg(Leg.Mode.FORWARD,35, 0,2.1));

    // Latch down here.
    Queue<Leg> LatchPt2 = new LinkedList<>();
    LatchPt2.add(new Leg(Leg.Mode.TURN, 50, 10, 0));
    LatchPt2.add(new Leg(Leg.Mode.BACKWARDS,35, 0,1.5));
    LatchPt2.add(new Leg(Leg.Mode.TURN, 50, 0, 0));
    LatchPt2.add(new Leg(Leg.Mode.BACKWARDS,35, 0,1.0));
    LatchPt2.add(new Leg(Leg.Mode.FORWARD,35, 0,0.3));
  
    // Latch up here.
    Queue<Leg> LatchPt3 = new LinkedList<>();
    LatchPt3.add(new Leg(Leg.Mode.BACKWARDS,35, 0,0.2));
    LatchPt3.add(new Leg(Leg.Mode.LEFT, 50, 0, 2.25));
    LatchPt3.add(new Leg(Leg.Mode.FORWARD,35, 0,0.5));
    LatchPt3.add(new Leg(Leg.Mode.LEFT, 50, 0, 1.3));
    
    // Wait for the game to start (driver presses PLAY)
    waitForStart();

    // for each piece of the drive & manipulate plan you will need to load a plan and then put
    // a while loop, like the following example, that repeatedly calls the autoDrive method
    // until the driving is done.
    // Put manipulator movements between the driving loops.
    // If you need more driving load another plan and make another loop.
    manipulatorPlatform.latchPosition(true);  // unlatch
    
    driveChassis.startPlan(LatchPt1);
    while (opModeIsActive() && driveChassis.isDriving())
    {
      // Process the drive chassis
      driveChassis.autoDrive(telemetry);
    }

    // After driving do your manipulation.  You may need a timer based state machine but simple
    // actions can just be done inline.
    manipulatorPlatform.latchPosition(false);  // latch
    // spin for a second to allow latches to move.
    manipulateimer.reset();
    while (opModeIsActive() && manipulateimer.time()< 1.0);


    // do second part of drive plan.
    driveChassis.startPlan(LatchPt2);
    while (opModeIsActive() && driveChassis.isDriving())
    {
      // Process the drive chassis
      driveChassis.autoDrive(telemetry);
    }

    // After driving do your manipulation.  You may need a timer based state machine but simple
    // actions can just be done inline.
    manipulatorPlatform.latchPosition(true);  // unlatch
    // spin for a second to allow latches to move.
    manipulateimer.reset();
    while (opModeIsActive() && manipulateimer.time()< 1.0);

    // do third part of drive plan.
    driveChassis.startPlan(LatchPt3);
    while (opModeIsActive() && driveChassis.isDriving())
    {
      // Process the drive chassis
      driveChassis.autoDrive(telemetry);
    }
  }
}
