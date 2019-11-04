package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@TeleOp(name = "Test_TheIMU", group = "Sensor")
// @Disabled                            // Comment this out to add to the opmode list
public class Test_TheIMU extends LinearOpMode
{
  //----------------------------------------------------------------------------------------------
  // State
  //----------------------------------------------------------------------------------------------

  // The IMU sensor object
  private Team9013IMU imu;

  // State used for updating telemetry
  private Orientation angles;
  private Acceleration acceleration;
  private Position position;

  //----------------------------------------------------------------------------------------------
  // Main logic
  //----------------------------------------------------------------------------------------------

  @Override public void runOpMode()
  {

    // Set up the parameters with which we will use our IMU. Note that integration
    // algorithm here just reports accelerations to the logcat log; it doesn't actually
    // provide positional information.
    Team9013IMU.Parameters parameters = new Team9013IMU.Parameters();
    parameters.angleUnit           = Team9013IMU.AngleUnit.DEGREES;
    parameters.accelUnit           = Team9013IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.calibrationDataFile = "IMUCalibration.json"; // see the calibration sample opmode
    parameters.loggingEnabled      = false;
    parameters.loggingTag          = "IMU";
    parameters.accelerationIntegrationAlgorithm = new AccelerationIntegrator9013();

    // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
    // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
    // and named "imu".
    imu = hardwareMap.get(Team9013IMU.class, "imu");

    imu.initialize(parameters);

    // make sure the gyro is calibrated before continuing
    /* ToDo Figure out why/how calibration data file is not working showing the gyro and accel are
        calibrated.  */
//    while (!imu.isGyroCalibrated() || !imu.isAccelerometerCalibrated())
//    while (!imu.isGyroCalibrated())
//    {
//      sleep(50);
//    }

    // Set up our telemetry dashboard
    composeTelemetry();

    // Wait until we're told to go
    waitForStart();

    // Start the integrating acceleration
    imu.startAccelerationIntegration(new Position(), new Velocity(), 50);

    // Loop and update the dashboard
    while (opModeIsActive()) {
      telemetry.update();
    }
  }

  //----------------------------------------------------------------------------------------------
  // Telemetry Configuration
  //----------------------------------------------------------------------------------------------

  void composeTelemetry()
  {

    // At the beginning of each telemetry update, grab a bunch of data
    // from the IMU that we will then display in separate lines.
    telemetry.addAction(new Runnable() { @Override public void run()
    {
      // Acquiring the angles is relatively expensive; we don't want
      // to do that in each of the three items that need that info, as that's
      // three times the necessary expense.
      angles        = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
      acceleration  = imu.getAcceleration();
      position      = imu.getPosition();

    }
    });

    telemetry.addLine()
        .addData("status", new Func<String>() {
          @Override public String value() {
            return imu.getSystemStatus().toShortString();
          }
        })
        .addData("calib", new Func<String>() {
          @Override public String value() {
            return imu.getCalibrationStatus().toString();
          }
        });

    telemetry.addLine()
        .addData("heading", new Func<String>() {
          @Override public String value() {
            return formatAngle(angles.angleUnit, angles.firstAngle);
          }
        })
        .addData("roll", new Func<String>() {
          @Override public String value() {
            return formatAngle(angles.angleUnit, angles.secondAngle);
          }
        })
        .addData("pitch", new Func<String>() {
          @Override public String value() {
            return formatAngle(angles.angleUnit, angles.thirdAngle);
          }
        });

    telemetry.addLine()
        .addData("accel", new Func<String>() {
          @Override public String value() {
            return acceleration.toString();
          }
        })
        .addData("mag", new Func<String>() {
          @Override public String value() {
            return String.format(Locale.getDefault(), "%.3f",
                Math.sqrt(acceleration.xAccel*acceleration.xAccel
                    + acceleration.yAccel*acceleration.yAccel
                    + acceleration.zAccel*acceleration.zAccel));
          }
        });
    telemetry.addLine()
        .addData("pos.", new Func<String>() {
          @Override public String value() {
            return position.toString();
          }
        });
  }

  //----------------------------------------------------------------------------------------------
  // Formatting
  //----------------------------------------------------------------------------------------------

  String formatAngle(AngleUnit angleUnit, double angle) {
    return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
  }

  String formatDegrees(double degrees){
    return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
  }
}
