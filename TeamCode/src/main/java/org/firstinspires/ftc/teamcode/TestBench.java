package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TestBench", group="Linear Opmode")
//@Disabled
public class TestBench extends LinearOpMode {

  // Declare OpMode members.
  private TestBenchPlatform testBench;

  private ElapsedTime runtime = new ElapsedTime();

  @Override
  public void runOpMode() {
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    testBench = new TestBenchPlatform(hardwareMap);

    // Wait for the game to start (driver presses PLAY)
    waitForStart();
    runtime.reset();

    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {

      // send joystick inputs to the drive chassis
      testBench.test(gamepad1.left_stick_y, gamepad1.left_stick_x,
                         gamepad1.right_stick_x, telemetry);
      
    }
  }
}
