package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

public class ManipulatorPlatform
{
  private DcMotorEx gatherLeftMotor = null;
  private DcMotorEx gatherRightMotor = null;
  private DcMotor elevatorMotor = null;
  private DcMotor extenderMotor = null;
 
  private Servo latchLeftServo = null;
  private Servo latchRightServo = null;
  private Servo grabberServo = null;
  private Servo rotatorServo = null;
  
  ManipulatorPlatform(HardwareMap hardwareMap)
  {
    // Initialize the hardware variables. Note that the strings used here as parameters
    // to 'get' must correspond to the names assigned during the robot configuration
    // step (using the FTC Robot Controller app on the phone).

    latchLeftServo = hardwareMap.get(Servo.class, "lLServo");
    latchRightServo = hardwareMap.get(Servo.class, "rLServo");
    grabberServo = hardwareMap.get(Servo.class, "gServo");
    rotatorServo = hardwareMap.get(Servo.class, "rServo");

    gatherLeftMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "lGMotor");  //hub 2 port 0
    gatherRightMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "rGMotor");  //hub 2 port 0
    elevatorMotor = hardwareMap.get(DcMotor.class, "elMotor");  //hub 2 port 0
    extenderMotor = hardwareMap.get(DcMotor.class, "exMotor");  //hub 2 port 0

    gatherLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    gatherRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    gatherLeftMotor.setDirection(DcMotor.Direction.REVERSE);
    gatherRightMotor.setDirection(DcMotor.Direction.REVERSE);

    gatherLeftMotor.setVelocity(0, RADIANS); // radians/second
    gatherRightMotor.setVelocity(0, RADIANS);

    elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    elevatorMotor.setPower(0);
    elevatorMotor.setDirection(DcMotor.Direction.REVERSE);
    elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    elevatorMotor.setTargetPosition(0);
    elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    extenderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    extenderMotor.setPower(0);
    extenderMotor.setDirection(DcMotor.Direction.FORWARD);
    extenderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    extenderMotor.setTargetPosition(0);
    extenderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }


  void test(float driveLeftY, float driveLeftX, float driveRightX, Telemetry telemetry)
  {
 //   telemetry.addData("Heading (rad) ", " %.4f", IMUTelemetry.heading );
 //   telemetry.addData("Error (rad) ", " %.4f",IMUTelemetry.error );
    telemetry.update();

  }


  void gatherOn()
  {
    gatherLeftMotor.setVelocity(2 * Math.PI, RADIANS); // radians/second
    gatherRightMotor.setVelocity(2 * Math.PI, RADIANS);
  }


  void gatherOff()
  {
    gatherLeftMotor.setVelocity(0, RADIANS); // radians/second
    gatherRightMotor.setVelocity(0, RADIANS);
  }

  void elevatorPosition(int count)
  {
    elevatorMotor.setPower(1);
    elevatorMotor.setTargetPosition(count);
  }


  void extenderPosition(int count)
  {
    extenderMotor.setPower(1);
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


  void grab(boolean position)
  {
    if(position) // grab (1)
    {
      grabberServo.setPosition(1);
    }
    else
    {
      grabberServo.setPosition(0);
    }
  }


  void rotate(boolean position)
  {
    if(position) // rotated (1)
    {
      rotatorServo.setPosition(1);
    }
    else
    {
      rotatorServo.setPosition(0);
    }
  }
}
