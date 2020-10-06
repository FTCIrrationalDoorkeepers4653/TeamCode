package org.firstinspires.ftc.teamcode.Programs.DriverControl;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Systems.Movement.Robot;

@TeleOp(name = "VisionTest")
//@Disabled
public class VisionTest extends LinearOpMode {
  /* TEST VARIABLES */

  //Objects and Variables:
  Robot robot = new Robot();
  int position = 1;

  /* OPMODE METHODS */

  @Override
  public void runOpMode() {
    //Status Updates:
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    //Hardware Initialization:
    robot.init(hardwareMap, false);

    //Waits for Start:
    waitForStart();

    //Loops Until Stop:
    while (!isStopRequested()) {
      //Sets the Position:
      position = robot.getPixelsPosition();

      //Telemetry Data:
      telemetry.addData("Position", position);
      telemetry.update();
    }

    //Status Update:
    telemetry.addData("Status", "Stopped");
    telemetry.update();
  }
}
