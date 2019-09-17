package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Test_TheIMU", group = "Linear Opmode")

@Disabled
public class Test_TheIMU extends LinearOpMode
{

  // Declare OpMode members.
  private IMU_TEST imu;

  private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);


  //  @Override
  public void runOpMode()
  {

    imu = new IMU_TEST(hardwareMap);

    // Wait for the game to start (driver presses PLAY)
    waitForStart();
    runtime.reset();

    while (opModeIsActive())
    {
      IMUTelemetry telData = imu.testAngle(-120);

      telemetry.addData("Heading (deg) ", " %.2f", telData.heading );
      telemetry.addData("Error (deg) ", " %.2f",telData.error );
      telemetry.update();
    }
  }
}