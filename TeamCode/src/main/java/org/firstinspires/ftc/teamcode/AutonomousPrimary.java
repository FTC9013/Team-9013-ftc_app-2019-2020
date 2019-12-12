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
  private ManipulatorPlatform manipulatorPlatform;
  private LEDs leds;

  private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

  private static final float mmPerInch = 25.4f;
  private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
  private static final float mmTargetHeight = (6) * mmPerInch; // the height of the center of the target image above the floor

  private boolean extendedFlag = false;
  
  private final int extenderRetracted  = 0;
  private final int extenderExtended  = 850;

  @Override
  public void runOpMode() {

    driveChassis = new MecanumDriveChassisAutonomous(hardwareMap);
    manipulatorPlatform = new ManipulatorPlatform(hardwareMap);
    leds = new LEDs(hardwareMap);
    leds.goOff();
  
    // build all the drive plans for drive by distance (to move the gold mineral)
    //
    // Each leg of the trip is added to the queue in this code block.
    // As the opmode runs, the queue sent to the drive base for execution.
    //
    // mode:     {FORWARD, BACKWARDS, TURN_DRIVE}
    // speed:    the drive speed from 0-100%
    // angle:    Ignored in all but TURN mode:
    //           the desired angle of travel relative to the ZERO orientation in DEGREES
    //           ZERO is where the bot was facing when the IMU calibrated.
    //           desired angle in degrees 0 to 360 CCW
    // distance: Only used for FORWARD, BACKWARD modes:  the distance to travel in inches

    Queue<Leg> leftPath = new LinkedList<>();
    leftPath.add(new Leg(Leg.Mode.TURN, 100, 90, 0));
    leftPath.add(new Leg(Leg.Mode.TURN, 20, 0, 0));
    leftPath.add(new Leg(Leg.Mode.TURN, 100, 270, 0));
    leftPath.add(new Leg(Leg.Mode.FORWARD, 50, 0, 20));

    Queue<Leg> ParkWallSideDepot = new LinkedList<>();
    ParkWallSideDepot.add(new Leg(Leg.Mode.TURN,10, 90, 0));
    ParkWallSideDepot.add(new Leg(Leg.Mode.FORWARD,100, 0, 22 ));

    Queue<Leg> ParkBridgeSideDepot = new LinkedList<>();
    ParkBridgeSideDepot.add(new Leg(Leg.Mode.FORWARD,100,0,22));
    ParkBridgeSideDepot.add(new Leg(Leg.Mode.TURN,10,90,0));
    ParkBridgeSideDepot.add(new Leg(Leg.Mode.FORWARD,100,0,22));

    Queue<Leg> ParkWallSideBuild = new LinkedList<>();
    ParkWallSideBuild.add(new Leg(Leg.Mode.TURN,10, 270,0));
    ParkWallSideBuild.add(new Leg(Leg.Mode.TURN, 100,0,22));

    Queue<Leg> ParkBridgeSideBuild = new LinkedList<>();
    ParkBridgeSideBuild.add(new Leg(Leg.Mode.FORWARD,100,0,22));
    ParkBridgeSideBuild.add(new Leg(Leg.Mode.TURN,10,270,0));
    ParkBridgeSideBuild.add(new Leg(Leg.Mode.FORWARD,100,0,22));

    // Wait for the game to start (driver presses PLAY)
    waitForStart();
    runtime.reset();

    // load the path
    driveChassis.startPlan(ParkBridgeSideBuild);


    while (opModeIsActive())
    {
      // Process the drive chassis
      driveChassis.autoDrive( telemetry );
      
      if(runtime.time() > 20 && !extendedFlag)
      {
        extendedFlag = true;
        manipulatorPlatform.extenderPosition(extenderExtended);
      }
    }
    
  }
}
