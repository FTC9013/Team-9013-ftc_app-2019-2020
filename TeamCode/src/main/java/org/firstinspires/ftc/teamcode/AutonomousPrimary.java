package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

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
    //           desired angle in degrees +/- 0 to 180 where CCW is + and CW is -
    // distance: the distance to travel in inches

    Queue<Leg> leftPath = new LinkedList<>();
    leftPath.add(new Leg(Leg.Mode.TURN_DRIVE, 40, 0, 0));
    leftPath.add(new Leg(Leg.Mode.FORWARD, 40, 0, 20));
    leftPath.add(new Leg(Leg.Mode.LEFT, 40, 0, 13));
    leftPath.add(new Leg(Leg.Mode.FORWARD, 40, 0, 15));
    leftPath.add(new Leg(Leg.Mode.BACKWARDS, 40, 0, 15));
    leftPath.add(new Leg(Leg.Mode.TURN_DRIVE, 60, 40, 0));

    Queue<Leg> rightPath = new LinkedList<>();
    rightPath.add(new Leg(Leg.Mode.TURN_DRIVE, 40, 0, 0));
    rightPath.add(new Leg(Leg.Mode.FORWARD, 40, 0, 20));
    rightPath.add(new Leg(Leg.Mode.RIGHT, 40, 0, 20));
    rightPath.add(new Leg(Leg.Mode.FORWARD, 40, 0, 15));
    rightPath.add(new Leg(Leg.Mode.BACKWARDS, 40, 0, 15));
    rightPath.add(new Leg(Leg.Mode.TURN_DRIVE, 60, 40, 0));

    // Wait for the game to start (driver presses PLAY)
    waitForStart();
    runtime.reset();

    while (opModeIsActive())
    {






    }


  }
}
