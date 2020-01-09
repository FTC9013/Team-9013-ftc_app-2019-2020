package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.LinkedList;
import java.util.Queue;

@Autonomous(name = "AutonomousPrimary", group = "Linear Opmode")

//@Disabled
public class AutonomousPrimary extends LinearOpMode {

  // Declare OpMode members.
  private MecanumDriveChassisAutonomous driveChassis;
//  private ManipulatorPlatform manipulatorPlatform;
//  private LEDs leds;

  private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

  // save for later
  // private static final float mmPerInch = 25.4f;
  // private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
  // private static final float mmTargetHeight = (6) * mmPerInch; // the height of the center of the target image above the floor

  private boolean extendedFlag = false;
  
  private final int extenderRetracted  = 0;
  private final int extenderExtended  = 850;

  @Override
  public void runOpMode() {

    driveChassis = new MecanumDriveChassisAutonomous(hardwareMap);
//    manipulatorPlatform = new ManipulatorPlatform(hardwareMap);
//    leds = new LEDs(hardwareMap);
//    leds.goOff();
  
    // build all the drive plans for drive by distance (time in seconds)
    //
    // Each leg of the trip is added to the queue in this code block.
    // As the opmode runs, the queue sent to the drive base for execution.
    //
    // mode:     {FORWARD, BACKWARDS, LEFT (strafe), RIGHT (strafe), TURN}
    // speed:    the drive speed from 0-100%  (Slower speeds for longer times will be more precise)
    // angle:    Ignored in all but TURN mode:
    //           the desired angle of travel relative to the ZERO orientation in DEGREES
    //           ZERO is where the bot was facing when the IMU calibrated.
    //           desired angle in degrees 0 to 360 CCW
    // distance: Only used for FORWARD, BACKWARD, LEFT, RIGHT, modes:  the TIME in seconds to run
    //           the motors.

    Queue<Leg> TestAllFunctions = new LinkedList<>();
    TestAllFunctions.add(new Leg(Leg.Mode.FORWARD, 50, 0, 1));
    TestAllFunctions.add(new Leg(Leg.Mode.TURN, 100, 90, 0));
    TestAllFunctions.add(new Leg(Leg.Mode.FORWARD, 50, 0, 1));
    TestAllFunctions.add(new Leg(Leg.Mode.TURN, 100, 180, 0));
    TestAllFunctions.add(new Leg(Leg.Mode.FORWARD, 50, 0, 1));
    TestAllFunctions.add(new Leg(Leg.Mode.TURN, 100, 270, 0));
    TestAllFunctions.add(new Leg(Leg.Mode.FORWARD, 50, 0, 1));
    TestAllFunctions.add(new Leg(Leg.Mode.TURN, 100, 0, 0));

    //TestAllFunctions.add(new Leg(Leg.Mode.LEFT, 50, 0, 1));
    //TestAllFunctions.add(new Leg(Leg.Mode.BACKWARDS, 50, 0, 1));
    //TestAllFunctions.add(new Leg(Leg.Mode.RIGHT, 50, 0, 1));
    //TestAllFunctions.add(new Leg(Leg.Mode.TURN, 50, 0, 0));

// below are the different parking sequences
    Queue<Leg> Forward = new LinkedList<>();
    Forward.add(new Leg(Leg.Mode.FORWARD,20, 0, 2 ));

    Queue<Leg> Left = new LinkedList<>();
    Left.add(new Leg(Leg.Mode.LEFT,20, 0, 2 ));

    Queue<Leg> ForwardRight = new LinkedList<>();
    ForwardRight.add(new Leg(Leg.Mode.FORWARD,20,0,2));
    ForwardRight.add(new Leg(Leg.Mode.TURN,10,270,0));
    ForwardRight.add(new Leg(Leg.Mode.FORWARD,20,0,2));

    Queue<Leg> Right = new LinkedList<>();
    Right.add(new Leg(Leg.Mode.RIGHT,20, 0,2));

    Queue<Leg> ForwardLeft = new LinkedList<>();
    ForwardLeft.add(new Leg(Leg.Mode.FORWARD,20,0,2));
    ForwardLeft.add(new Leg(Leg.Mode.TURN,10,90,0));
    ForwardLeft.add(new Leg(Leg.Mode.FORWARD,20,0,2));

    // Wait for the game to start (driver presses PLAY)
    waitForStart();
    runtime.reset();

    // load the path
    driveChassis.startPlan(Forward);

    while (opModeIsActive())
    {
      // Process the drive chassis
      driveChassis.autoDrive( telemetry );

      // just an example, not likely the way one would use this in a state machine.
      if(!driveChassis.isDriving())
      {
        // do something once driving has stopped
      }
    }
  }
}
