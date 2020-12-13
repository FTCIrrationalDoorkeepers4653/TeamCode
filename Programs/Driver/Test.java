package org.firstinspires.ftc.teamcode.Programs.Driver;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Systems.Core.Robot;

@TeleOp(name="Test")
public class Test extends OpMode {
  /* TEST VARIABLES */

  //Objects and Variables:
  private Robot robot = new Robot();
  private int position = 1;

  /* OPMODE METHODS */

  //Init Method:
  @Override
  public void init() {
    //Status Updates:
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    //Hardware Initialization:
    robot.init(hardwareMap, false, true);
  }

  //Loop Method:
  @Override
  public void loop() {
    //Gets the Position:
    position = robot.getPixelsPosition();

    //Status Updates:
    telemetry.addData("Position", position);
    telemetry.update();
  }

  //Stop Method:
  @Override
  public void stop() {
    //Status Updates:
    telemetry.addData("Status", "Stopped");
    telemetry.update();
  }
}

