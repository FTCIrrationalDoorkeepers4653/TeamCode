package org.firstinspires.ftc.teamcode.Programs.Driver;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Systems.Core.Robot;

@TeleOp(name = "VisionOp")
public class VisionOp extends LinearOpMode {
  /* VISION VARIABLES */

  //Objects and Variables:
  private Robot robot = new Robot();
  private int position = 1;

  /* OPMODE METHODS */

  @Override
  public void runOpMode() {
    //Status Updates:
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    //Hardware Initialization:
    robot.init(hardwareMap, false, true);

    //Waits for Start:
    waitForStart();

    //Loops Until Stop:
    mainLoop: while (opModeIsActive()) {
      //Gets the Position:
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
