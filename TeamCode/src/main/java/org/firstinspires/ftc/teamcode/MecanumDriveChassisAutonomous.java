package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.math.BigDecimal;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Queue;

import static java.lang.Math.abs;
import static java.lang.Thread.sleep;

public class MecanumDriveChassisAutonomous
{
  private DcMotor leftFrontDrive = null;
  private DcMotor leftRearDrive = null;
  private DcMotor rightFrontDrive = null;
  private DcMotor rightRearDrive = null;

  private BNO055IMU imu;
  private static IMUTelemetry IMUTelemetry;
  private Orientation angles; // stores the current orientation of the bot from the IMU

  private static double leftFrontDriveSpeed;
  private static double leftRearDriveSpeed;
  private static double rightFrontDriveSpeed;
  private static double rightRearDriveSpeed;

  // Robot speed [-1, 1].  (speed in any direction that is not rotational)
  // does not have any angular component, just scaler velocity.
  // combined with the angular component for motion.  Even if angle is 0 (forward).
  private static double vD;

  // Robot angle while moving [0, 2PI] or [0, +/-PI]. (angle to displace the center of the bot,
  // ASDF)
  // relative to the direction the bot is facing.
  private static double thetaD;

  // Speed component for rotation about the Z axis. [-x, x]
  private static double vTheta;

  // heading about a unit circle in radians.
  private static double desiredHeading;  // rotates about the Z axis [0,2PI) rad.
  private static double currentHeading;  // rotates about the Z axis [0,2PI) rad.

  // Robot speed scaling factor (% of joystick input to use)
  // applied uniformly across all joystick inputs to the JoystickToMotion() method.
  private final double speedScale = 1.0;

  // PID for the heading
  private final double propCoeff = 0.9;
  private final double integCoeff = 0.0;
  private final double diffCoeff = 0.00;
  private final double OutputLowLimit = -1;
  private final double OutputHighLimit = 1;
  private final double MaxIOutput = 1;
  private final double OutputRampRate = 0.1;
  private final double OutputFilter = 0;
  private final double SetpointRange = 2*Math.PI;

  private final double headingThreshold = 0.05;
  private final int headdingAverageNumberOfSamples = 10;

  private PID headingPID = null;
  private RollingAverage averageHeading = null;

  MecanumDriveChassisAutonomous(HardwareMap hardwareMap)
  {
    // Initialize the hardware variables. Note that the strings used here as parameters
    // to 'get' must correspond to the names assigned during the robot configuration
    // step (using the FTC Robot Controller app on the phone).
    leftFrontDrive = hardwareMap.get(DcMotor.class, "lFront"); //hub 3 port 0
    leftRearDrive = hardwareMap.get(DcMotor.class, "lRear"); //hub 3 port 2
    rightFrontDrive = hardwareMap.get(DcMotor.class, "rFront"); //hub 3 port 1
    rightRearDrive = hardwareMap.get(DcMotor.class, "rRear"); //hub 3 port 3

    // Get and initialize the IMU. (we will use the imu on hub id = 3)
    imu = hardwareMap.get(BNO055IMU.class, "imu");

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    IMUTelemetry = new IMUTelemetry();

    // set the initial imu mode parameters.
    parameters.mode = BNO055IMU.SensorMode.IMU;
    parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
    parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.loggingEnabled      = false;

    imu.initialize(parameters);

    leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    // Motors on one side reversed to drive forward
    // Reverse the motor that runs backwards when connected directly to the battery
    // A positive power number should drive the robot forward regardless of the motor's
    // position on the robot.
    leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
    leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
    rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
    rightRearDrive.setDirection(DcMotor.Direction.FORWARD);

    // set motion parameters.
    vD = 0;
    thetaD = 0;
    vTheta = 0;

    // Set all the motor speeds.
    rightFrontDriveSpeed = 0;
    leftFrontDriveSpeed = 0;
    rightRearDriveSpeed = 0;
    leftRearDriveSpeed = 0;

    rightFrontDrive.setPower(rightFrontDriveSpeed);
    leftFrontDrive.setPower(leftFrontDriveSpeed);
    rightRearDrive.setPower(rightRearDriveSpeed);
    leftRearDrive.setPower(leftRearDriveSpeed);

    // make sure the gyro is calibrated before continuing
    while (!imu.isGyroCalibrated())
    {
      try {
        sleep(50);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
  
    // create and initialize the PID for the heading
    headingPID = new PID(propCoeff, integCoeff, diffCoeff);

    // get the initial error and put valid data in the telemetry from the imu
    testAngle();

    // set initial desired heading to the current actual heading.
    desiredHeading = currentHeading;

    // smooths out the joystick input so it doesn't slam hi/lo
    averageHeading = new RollingAverage(headdingAverageNumberOfSamples);

    // initially setup the PID parameters
    headingPID.setOutputLimits( OutputLowLimit, OutputHighLimit);
    headingPID.setMaxIOutput(MaxIOutput);
    headingPID.setOutputRampRate(OutputRampRate);
    headingPID.setOutputFilter(OutputFilter);
    headingPID.setSetpointRange(SetpointRange);
    headingPID.setContinousInputRange(2*Math.PI);
    headingPID.setContinous(true);  // lets PID know we are working with a continuous range [0-360)
  }

  // Left  Y = forward, backward movement
  // Left  X = side to side (strafe)
  // Right X = rotate in place
  void drive(float driveLeftY, float driveLeftX, float driveRightX, Telemetry telemetry)
  {
    telemetry.addData("Heading (rad) ", " %.4f", IMUTelemetry.heading );
    telemetry.addData("Error (rad) ", " %.4f",IMUTelemetry.error );
    telemetry.update();

    // calculate the vectors multiply input values by scaling factor for max speed.
    joystickToMotion( driveLeftY * speedScale, driveLeftX * speedScale,
        driveRightX * speedScale  );

    // Math out what to send to the motors and send it.
    PowerToWheels();
  }

  /**
   * Process the joystick values into motion vector.
   *  Converts the left stick X, Y input into the translation angle and speed
   *  Converts the right stick X axis into rotation speed.
   *
   *  Overall this makes the joysticks like mouse & keyboard game controls with
   *  the left stick acting as the WASD keys and the right stick as the mouse.
   *
   *  vD = desired robot translation speed.
   *  thetaD = desired robot translation angle.
   *  vTheta = desired robot rotational speed.
   */
  private void joystickToMotion( double leftStickY, double leftStickX, double rightStickX ) {
    // determines the translation speed by taking the hypotenuse of the vector created by
    // the X & Y components.
    vD = Math.min(Math.sqrt(Math.pow(leftStickX, 2) + Math.pow(-leftStickY, 2)), 1);

    // Converts the joystick inputs from cartesian to polar from 0 to +/- PI oriented
    // with 0 to the right of the robot. (standard polar plot)
    thetaD = Math.atan2(-leftStickY, leftStickX);
    // orient to the robot by rotating PI/2 to make the joystick zero at the forward of bot.
    // instead of the right side.
    thetaD = thetaD - Math.PI / 2;
    // simply takes the right stick X value and invert to use as a rotational speed.
    // inverted since we want CW rotation on a positive value.
    // which is opposite of what PowerToWheels() wants in polar positive rotation (CCW).
//    vTheta = -rightStickX;

    // if there is new joystick input update the heading otherwise hold the current heading as
    // the setpoint.
    // headding is in radians so just using the +/- 1 from the joystick to add as a bias to the
    // current angle will put the desired head +/- 57 degrees from current.  This should be more
    // than enough to move the bot at max rotation speed.
    // The chasing of this setpoint is controled by the PID loop on the vTheta value.
  
  
    averageHeading.add(rightStickX);  // average in the current stick value

    // if the averaged stick input is greater then the headingThreshold go ahead and adjust the heading.
    // This keeps from updating the desiredHeading value if no joystick input is being made.
    // Otherwise, it will always drive the desiredHeading to 0 (neutral joystick position)
    if(Math.abs(averageHeading.getAverage()) > headingThreshold)
    {
      desiredHeading = currentHeading - averageHeading.getAverage();
      // keep heading a positive angle
      if (desiredHeading < 0)
      {
        desiredHeading += (2 * Math.PI);
      }
    }
    // get the imu angles in the format we need.
    testAngle();

    // PID controls the vTheta input to the wheel power equation.
    // vTheta = headingPID.getOutput(currentHeading, desiredHeading );
  }

  /**
   * Calculate the power settings and send to the wheels.  This also translates the force
   * for the Mecanum wheels to the rotated axis based on the degree of the wheel offset.
   * In our case 45 degrees or PI/4
   *
   * Assumes X is forward and Z is up then rotate XY PI/4 to align with wheel axises.
   * placing the positive X axis on the left front wheel and the positive Y axis on the
   * left rear wheel.
   *
   * Rotation is about a positive Z axis pointing UP.
   * Positive Y is to the left.
   *
   * Translation angle is in radians + is CCW - is CW with ZERO to the forward of the bot.
   * I.e. standard rotation about a positive Z axis pointing UP.
   * E.g:
   *    0     = forward
   *    PI/4  = 45 deg. forward and to the left
   *    PI/2  = to the left
   *    3PI/4 = 135 deg. backward and to the left
   *    PI    = backwards
   *
   *    -PI/4  (or) 7PI/4 = 45 deg. forward and to the right
   *    -PI/2  (or) 6PI/4 = to the right
   *    -3PI/4 (or) 5PI/4 = -135 deg. backward and to the left
   *    -PI    (or) PI    = backwards
   *
   * vTheta rotation is also standard rotation about a positive Z axis pointing UP.
   * thus a positive vTheta will turn the bot CCW about its Z axis.
   *
   **/
  private void PowerToWheels() {

    // Motors power = Y component of directional vector
    leftFrontDriveSpeed  = vD * Math.sin(-thetaD + Math.PI / 4) - vTheta;
    rightRearDriveSpeed  = vD * Math.sin(-thetaD + Math.PI / 4) + vTheta;

    // Motors power = X component of directional vector
    rightFrontDriveSpeed = vD * Math.cos(-thetaD + Math.PI / 4) + vTheta;
    leftRearDriveSpeed   = vD * Math.cos(-thetaD + Math.PI / 4) - vTheta;

    // place all the power numbers in a list for collection manipulations
    // (easier to find min / max etc when in a list)
    List<Double> speeds = Arrays.asList(rightFrontDriveSpeed,
        leftFrontDriveSpeed, rightRearDriveSpeed, leftRearDriveSpeed  );

    // scales the motor powers while maintaining power ratios.
    double minPower = Collections.min(speeds);
    double maxPower = Collections.max(speeds);
    double maxMag = Math.max(Math.abs(minPower), Math.abs(maxPower));
    if (maxMag > 1.0)
    {
      for (int i = 0; i < speeds.size(); i++)
      {
        speeds.set(i, speeds.get(i) / maxMag);
      }
    }
    // must be same order as placed in the list
    // send the speeds to the motors
    rightFrontDrive.setPower(speeds.get(0));
    leftFrontDrive.setPower(speeds.get(1));
    rightRearDrive.setPower(speeds.get(2));
    leftRearDrive.setPower(speeds.get(3));
  }


  // grab the imu heading and crunch out the values used for navigation and telemetry.
  private void testAngle()
  {
    // desired angle in degrees +/- 0 to 180 where CCW is + and CW is -
    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

    // convert  imu angle range to our [0, 2PI) range
    if (angles.firstAngle < 0 )
    {
      currentHeading = angles.firstAngle + 2 * Math.PI;
    }
    else
    {
      currentHeading = angles.firstAngle;
    }

    IMUTelemetry.heading = currentHeading;
    IMUTelemetry.error = vTheta = headingPID.getOutput(currentHeading, desiredHeading );
  }


/*

  private Orientation angles; // stores the current orientation of the bot from the IMU

  private static double leftFrontDriveSpeed;
  private static double leftRearDriveSpeed;
  private static double rightFrontDriveSpeed;
  private static double rightRearDriveSpeed;

  private static IMUTelemetry IMUTel;


  // speed for am IMU turn
  private final double MaxTurnSpeed = 0.2;
  private final int countsPerTurnDegree = 18;
  private final double angleError = 2;          // turn angle error cutoff to stop turning.
  private final int countsPerDriveInch = 10000/117;
  private final int countsStrafePerInch = 5000/51;
  // +/- counts to say the motor is stopped a the target position.
  static final int closeEnough = 5;
  MecanumDriveChassisAutonomousIMU(HardwareMap hardwareMap)
  {

    // Initialize the hardware variables. Note that the strings used here as parameters
    // to 'get' must correspond to the names assigned during the robot configuration
    // step (using the FTC Robot Controller app on the phone).
    leftFrontDrive = hardwareMap.get(DcMotor.class, "lFront"); //hub 3 port 0
    leftRearDrive = hardwareMap.get(DcMotor.class, "lRear"); //hub 3 port 2
    rightFrontDrive = hardwareMap.get(DcMotor.class, "rFront"); //hub 3 port 1
    rightRearDrive = hardwareMap.get(DcMotor.class, "rRear"); //hub 3 port 3

    leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    // Motors on one side reversed to drive forward
    // Reverse the motor that runs backwards when connected directly to the battery
    // A positive power number should drive the robot forward regardless of the motor's
    // position on the robot.
    leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
    leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
    rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
    rightRearDrive.setDirection(DcMotor.Direction.FORWARD);

    // Set all the motor speeds to 0.
    rightFrontDriveSpeed = 0;
    leftFrontDriveSpeed = 0;
    rightRearDriveSpeed = 0;
    leftRearDriveSpeed = 0;

    rightFrontDrive.setPower(rightFrontDriveSpeed);
    leftFrontDrive.setPower(leftFrontDriveSpeed);
    rightRearDrive.setPower(rightRearDriveSpeed);
    leftRearDrive.setPower(leftRearDriveSpeed);

    // reset encoders and target positions to 0
    leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    leftFrontDrive.setTargetPosition(0);
    leftRearDrive.setTargetPosition(0);
    rightFrontDrive.setTargetPosition(0);
    rightRearDrive.setTargetPosition(0);

    leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    // Get and initialize the IMU. (we will use the imu on hub id = 3)
    imu = hardwareMap.get(BNO055IMU.class, "imu1");

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    // set the initial imu mode parameters.
    parameters.mode = BNO055IMU.SensorMode.IMU;
    // Set up the parameters with which we will use our IMU. Note that integration
    // algorithm here just reports accelerations to the logcat log; it doesn't actually
    // provide positional information.
    parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
    parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//    parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
    parameters.loggingEnabled      = false;
//    parameters.loggingTag          = "IMU";
    parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

    imu.initialize(parameters);

    IMUTel = new IMUTelemetry();
  }

  boolean IMU_IsCalibrated () {
    return imu.isGyroCalibrated();
  }

  /*
   * returns ture if the bot is still moving
   */

  /*
boolean isMoving() {
  return abs( rightFrontDrive.getCurrentPosition()
      - rightFrontDrive.getTargetPosition()) > closeEnough;

/*
    return rightFrontDrive.isBusy() || leftFrontDrive.isBusy()
           || rightRearDrive.isBusy()
           || leftRearDrive.isBusy();
*/

    /*
}

  // execute a path (list of drive legs)
  void move(Queue<Leg> path ) {
    Leg currentLeg;
    // as long as there are legs to drive...
    while(path.size() !=0 ){
      currentLeg = path.remove();
      // speed is passed in as a % from 1 to 100.  motor speed control range is 0-1 with
      // 1 being 100%
      setMoveSpeed(currentLeg.speed/100);

      switch (currentLeg.mode) {
        case FORWARD:
          driveForward(currentLeg.distance);
          break;

        case BACKWARDS:
          driveBackwards(currentLeg.distance);
          break;

        case LEFT:
          strafeLeft(currentLeg.distance);
          break;

        case RIGHT:
          strafeRight(currentLeg.distance);
          break;

        case TURN_DRIVE:
          setMoveSpeed(MaxTurnSpeed);
          turnToAbsoluteAngle(currentLeg.angle);
          setMoveSpeed(currentLeg.speed/100);
          driveForward(currentLeg.distance);
          break;
      }
    }
    // set speed back to 0 at the end of the drive
    setMoveSpeed(0);

  }

  void driveForward(double inches){
    // Move forward
    stopAndResetEncoders();
    leftFrontDrive.setTargetPosition((int)(inches * countsPerDriveInch));
    leftRearDrive.setTargetPosition((int)(inches * countsPerDriveInch));
    rightFrontDrive.setTargetPosition((int)(inches * countsPerDriveInch));
    rightRearDrive.setTargetPosition((int)(inches * countsPerDriveInch));
    runToPosition();
    // wait for motors to stop
    while (isMoving());
  }

  void driveBackwards(double inches){
    stopAndResetEncoders();
    leftFrontDrive.setTargetPosition(-(int)(inches * countsPerDriveInch));
    leftRearDrive.setTargetPosition(-(int)(inches * countsPerDriveInch));
    rightFrontDrive.setTargetPosition(-(int)(inches * countsPerDriveInch));
    rightRearDrive.setTargetPosition(-(int)(inches * countsPerDriveInch));
    runToPosition();
    // wait for motors to stop
    while (isMoving());
  }

  void strafeLeft(double inches){
    stopAndResetEncoders();
    leftFrontDrive.setTargetPosition(-(int)(inches * countsStrafePerInch));
    leftRearDrive.setTargetPosition((int)(inches * countsStrafePerInch));
    rightFrontDrive.setTargetPosition((int)(inches * countsStrafePerInch));
    rightRearDrive.setTargetPosition(-(int)(inches * countsStrafePerInch));
    runToPosition();
    // wait for motors to stop
    while (isMoving());
  }

  void strafeRight(double inches){
    stopAndResetEncoders();
    leftFrontDrive.setTargetPosition((int)(inches * countsStrafePerInch));
    leftRearDrive.setTargetPosition(-(int)(inches * countsStrafePerInch));
    rightFrontDrive.setTargetPosition(-(int)(inches * countsStrafePerInch));
    rightRearDrive.setTargetPosition((int)(inches * countsStrafePerInch));
    runToPosition();
    // wait for motors to stop
    while (isMoving());
  }

  void turnToAbsoluteAngle(double desiredAngle){
    double delta;
    do
    {
      stopAndResetEncoders();
      // desired angle in degrees +/- 0 to 180 where CCW is + and CW is -
      angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

      delta = desiredAngle - angles.firstAngle;
      if(delta > 180 ) {delta -= 360;}
      if(delta < -180 ) {delta += 360;}

      // as long as the bot is not on the desired heading, keep turning
      leftFrontDrive.setTargetPosition((int) (countsPerTurnDegree * -delta));
      leftRearDrive.setTargetPosition((int) (countsPerTurnDegree * -delta));
      rightFrontDrive.setTargetPosition((int) (countsPerTurnDegree * delta));
      rightRearDrive.setTargetPosition((int) (countsPerTurnDegree * delta));
      runToPosition();
      // wait for motors to stop
      while (isMoving());
    } while (Math.abs(delta) > angleError );  // keep going till close enough to desired angle
  }

  void setMoveSpeed(double speed) {
    rightFrontDrive.setPower(speed);
    leftFrontDrive.setPower(speed);
    rightRearDrive.setPower(speed);
    leftRearDrive.setPower(speed);
  }

  void stopAndResetEncoders(){
    leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  }

  void runToPosition() {
    leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }

 */


}
