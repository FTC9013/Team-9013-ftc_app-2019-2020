package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TestBenchPlatform
{
  private DcMotor leftFrontDrive = null;
  private DcMotor leftRearDrive = null;
  private DcMotor rightFrontDrive = null;
  private DcMotor rightRearDrive = null;

  private RevBlinkinLedDriver ledServo;
  private LEDs  leds = null;

  TestBenchPlatform(HardwareMap hardwareMap)
  {
    // Initialize the hardware variables. Note that the strings used here as parameters
    // to 'get' must correspond to the names assigned during the robot configuration
    // step (using the FTC Robot Controller app on the phone).
//    leftFrontDrive = hardwareMap.get(DcMotor.class, "lFront"); //hub 3 port 0
//    leftRearDrive = hardwareMap.get(DcMotor.class, "lRear"); //hub 3 port 2
//    rightFrontDrive = hardwareMap.get(DcMotor.class, "rFront"); //hub 3 port 1
//    rightRearDrive = hardwareMap.get(DcMotor.class, "rRear"); //hub 3 port 3

    // Get blinkin LED controller (emulates a servo...)
    ledServo = hardwareMap.get(RevBlinkinLedDriver.class, "leds");
    
    // create the LED object
    leds = new LEDs(ledServo);
    
  }

  // Left  Y = forward, backward movement
  // Left  X = side to side (strafe)
  // Right X = rotate in place
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
