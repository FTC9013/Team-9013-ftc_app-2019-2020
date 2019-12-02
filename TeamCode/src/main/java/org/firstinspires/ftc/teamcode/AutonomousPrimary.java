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

  private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

  private static final float mmPerInch = 25.4f;
  private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
  private static final float mmTargetHeight = (6) * mmPerInch; // the height of the center of the target image above the floor



  @Override
  public void runOpMode() {

    driveChassis = new MecanumDriveChassisAutonomous(hardwareMap);

    // build all the drive plans for drive by distance (to move the gold mineral)
    //
    // Each leg of the trip is added to the queue in this code block.
    // As the opmode runs, the queue sent to the drive base for execution.
    //
    // mode:     {FORWARD, BACKWARDS, LEFT, RIGHT, TURN_DRIVE}
    // speed:    the drive speed from 0-100%
    // angle:    the desired angle of travel relative to the ZERO orientation in DEGREES
    //           ZERO is where the bot was facing when the IMU calibrated.
    //           desired angle in degrees 0 to 360 CCW
    // distance: the distance to travel in inches

    Queue<Leg> leftPath = new LinkedList<>();
    leftPath.add(new Leg(Leg.Mode.TURN_DRIVE, 40, 0, 0));
    leftPath.add(new Leg(Leg.Mode.FORWARD, 40, 0, 20));
    leftPath.add(new Leg(Leg.Mode.TURN_DRIVE, 60, 90, 0));
    leftPath.add(new Leg(Leg.Mode.BACKWARDS, 40, 0, 15));




    // Wait for the game to start (driver presses PLAY)
    waitForStart();
    runtime.reset();

    // load the path
    driveChassis.startPlan(leftPath);


    while (opModeIsActive())
    {

      // Process the drive chassis
      driveChassis.autoDrive( telemetry );


    }


  }
}
