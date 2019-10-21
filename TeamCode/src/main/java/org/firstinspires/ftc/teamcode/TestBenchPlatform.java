package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TestBenchPlatform
{
  private DcMotor gatherLeftMotor = null;
  private DcMotor gatherRightMotor = null;
  private DcMotor elevatorMotor = null;
  private DcMotor extenderMotor = null;
 
  private Servo latchLeftServo = null;
  private Servo latchRightServo = null;
  private Servo grabberServo = null;
  private Servo grabberOrientationServo = null;
  
  private RevBlinkinLedDriver ledServo;
  private LEDs  leds = null;

  TestBenchPlatform(HardwareMap hardwareMap)
  {
    // Initialize the hardware variables. Note that the strings used here as parameters
    // to 'get' must correspond to the names assigned during the robot configuration
    // step (using the FTC Robot Controller app on the phone).

    // Get blinkin LED controller (emulates a servo...)
    ledServo = hardwareMap.get(RevBlinkinLedDriver.class, "leds");
  
  
    // Initialize the hardware variables. Note that the strings used here as parameters
    // to 'get' must correspond to the names assigned during the robot configuration
    // step (using the FTC Robot Controller app on the phone).
//    elevatorMotor = hardwareMap.get(DcMotor.class, "eMotor");  //hub 2 port 0
//
//    elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//    elevatorMotor.setDirection(DcMotor.Direction.FORWARD);
//
//    elevatorMotor.setPower(elevatorStop);
//
//    elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//    elevatorMotor.setPower(elevatorRun);
//
//    elevatorMotor.setTargetPosition(lobbyFloor);
//
//    elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//    gatherLeftMotor = hardwareMap.get(DcMotor.class, "gLMotor");
//
//    gatherLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//    gatherLeftMotor.setPower(elevatorStop);
//
//    gatherLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//    gatherLeftMotor.setPower(elevatorRun);
//
//    gatherLeftMotor.setTargetPosition(lobbyFloor);
//
//    gatherLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//    gatherRightMotor = hardwareMap.get(DcMotor.class, "aRMotor");
//
//    gatherRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//    gatherRightMotor.setPower(elevatorStop);
//
//    gatherRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//    gatherRightMotor.setPower(elevatorRun);
//
//    gatherRightMotor.setTargetPosition(lobbyFloor);
//
//    gatherRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//    extenderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//    extenderMotor.setPower(elevatorStop);
//
//    extenderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//    extenderMotor.setPower(elevatorRun);
//
//    extenderMotor.setTargetPosition(lobbyFloor);
//
//    extenderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    // create the LED object
    leds = new LEDs(ledServo);
    
  }


  void test(float driveLeftY, float driveLeftX, float driveRightX, Telemetry telemetry)
  {
 //   telemetry.addData("Heading (rad) ", " %.4f", IMUTelemetry.heading );
 //   telemetry.addData("Error (rad) ", " %.4f",IMUTelemetry.error );
    telemetry.update();



    


  }

  // grab the imu heading and crunch out the values used for navigation and telemetry.
  private void testAngle()
  {
  
  }
}
