package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.LinkedList;
import java.util.Queue;

@Autonomous(name = "AutoStrafeLeftFwd", group = "Linear Opmode")

//@Disabled
public class AutoStrafeLeftFwd extends LinearOpMode {

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
    private final int extenderExtended  = 700;

    @Override
    public void runOpMode() {

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


        Queue<Leg> ParkBridgeSideLeft = new LinkedList<>();
        ParkBridgeSideLeft.add(new Leg(Leg.Mode.FORWARD,20,0,2));
        ParkBridgeSideLeft.add(new Leg(Leg.Mode.TURN,10,90,0));
        ParkBridgeSideLeft.add(new Leg(Leg.Mode.FORWARD,20,0,2));


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // for each piece of the drive & manipulate plan you will need to load a plan and then put
        // a while loop, like the following example, that repeatedly calls the autoDrive method
        // until the driving is done.
        // Put manipulator movements between the driving loops.
        // If you need more driving load another plan and make another loop.

        driveChassis.startPlan(ParkBridgeSideLeft);
        while (opModeIsActive() && !driveChassis.isDriving())
        {
            // Process the drive chassis
            driveChassis.autoDrive(telemetry);
        }
    }
}