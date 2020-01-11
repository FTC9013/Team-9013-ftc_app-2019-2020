package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.qualcomm.robotcore.hardware.MotorControlAlgorithm.PIDF;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

public class ManipulatorPlatform
{
  private DcMotorEx gatherLeftMotor = null;
  private DcMotorEx gatherRightMotor = null;
  private DcMotorEx elevatorMotor = null;
  private DcMotorEx extenderMotor = null;
 
  private Servo latchLeftServo = null;
  private Servo latchRightServo = null;
  private Servo grabberServo = null;
  private Servo rotatorServo = null;
  private Servo gatherTableServo = null;
  private Servo gatherReleaseServo = null;

  private RevTouchSensor stonePresentSensor = null;

  static final double elevatorP = 75;
  static final double elevatorI = 0;
  static final double elevatorD = 0;
  static final double elevatorF = 0;

  static final double extenderP = 60;
  static final double extenderI = 0;
  static final double extenderD = 0;
  static final double extenderF = 0;


  ManipulatorPlatform(HardwareMap hardwareMap)
  {
    // Initialize the hardware variables. Note that the strings used here as parameters
    // to 'get' must correspond to the names assigned during the robot configuration
    // step (using the FTC Robot Controller app on the phone).

    latchLeftServo = hardwareMap.get(Servo.class, "lLServo");
    latchRightServo = hardwareMap.get(Servo.class, "rLServo");

    grabberServo = hardwareMap.get(Servo.class, "gServo");
    rotatorServo = hardwareMap.get(Servo.class, "rServo");

    gatherTableServo = hardwareMap.get(Servo.class, "gtServo");

    gatherReleaseServo = hardwareMap.get(Servo.class, "grServo");

    stonePresentSensor = hardwareMap.get(RevTouchSensor.class,"spSensor");


    gatherLeftMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "lGMotor");
    gatherRightMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "rGMotor");

    gatherLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    gatherRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    gatherLeftMotor.setDirection(DcMotor.Direction.REVERSE);
    gatherRightMotor.setDirection(DcMotor.Direction.REVERSE);

    gatherLeftMotor.setVelocity(0, RADIANS); // radians/second
    gatherRightMotor.setVelocity(0, RADIANS);



    elevatorMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "elMotor");
    elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    elevatorMotor.setDirection(DcMotor.Direction.REVERSE);
    elevatorMotor.setPower(0);
    elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    elevatorMotor.setPower(1);
    elevatorMotor.setTargetPosition(0);
    PIDFCoefficients elevatorPIDNew = new PIDFCoefficients( elevatorP, elevatorI, elevatorD, elevatorF );
    elevatorMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, elevatorPIDNew);
    elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    extenderMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "exMotor");
    extenderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    extenderMotor.setDirection(DcMotor.Direction.FORWARD);
    extenderMotor.setPower(0);
    extenderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    extenderMotor.setPower(1);
    extenderMotor.setTargetPosition(0);
    PIDFCoefficients extenderPIDNew = new PIDFCoefficients( extenderP, extenderI, extenderD, extenderF );
    extenderMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, extenderPIDNew);
    extenderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

  }

  void gatherOn(boolean direction)  // true = suck up stones
  {
    if(direction) // spin to gather stones
    {
      gatherLeftMotor.setVelocity(4 * Math.PI, RADIANS); // radians/second
      gatherRightMotor.setVelocity(-4 * Math.PI, RADIANS);
    }
    else
    {
      gatherLeftMotor.setVelocity(-3 * Math.PI, RADIANS); // radians/second
      gatherRightMotor.setVelocity(3 * Math.PI, RADIANS);
    }
  }

  void gatherOff()
  {
    // squirt out the stone.  Run for some time then stop
    gatherLeftMotor.setVelocity(0, RADIANS); // radians/second
    gatherRightMotor.setVelocity(0, RADIANS);
  }

  void gatherDown()
  {
    //servo down
    gatherTableServo.setPosition(0);
  }


  void gatherUp()
  {
    //servo up
    gatherTableServo.setPosition(1);
  }

  void gatherHold()
  {
    //servo set to hold stone
    gatherReleaseServo.setPosition(0);
  }


  void gatherRelease()
  {
    //servo set to release stone
    gatherReleaseServo.setPosition(1);
  }

  void gatherAbort()
  {
    //motors stop
    gatherOff();
    //servo up
    gatherUp();
  }


  void elevatorPosition(int count)
  {
    elevatorMotor.setTargetPosition(count);
  }


  void extenderPosition(int count)
  {
    extenderMotor.setTargetPosition(count);
  }


  void latchPosition(boolean position)
  {
    if(position) // Closed (1)
    {
      latchRightServo.setPosition(1);
      latchLeftServo.setPosition(0);
    }
    else
    {
      latchRightServo.setPosition(0);
      latchLeftServo.setPosition(1);
    }
  }


  void grabberGrab()
  {
    grabberServo.setPosition(0);
  }


  void grabberRelease()
  {
    grabberServo.setPosition(1);
  }


  void rotate0()
  {
    rotatorServo.setPosition(1);
  }


  void rotate90()
  {
    rotatorServo.setPosition(0);
  }


  boolean stonePresent()
  {
    return stonePresentSensor.isPressed();
  }

}
