package org.firstinspires.ftc.teamcode.Programs.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Systems.Core.Robot;

@Autonomous(name = "Park")
public class Park extends LinearOpMode {
  /* PARK VARIABLES */

  //Movement Variables:
  private Robot robot = new Robot();
  private double startX = 652.0;
  private double startY = 760.0;

  /* OPMODE METHODS */

  @Override
  public void runOpMode() {
    /* Initialization */

    //Status Updates:
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    //Hardware Initialization:
    robot.init(hardwareMap, true, false);
    robot.mechanisms.initMechanisms(hardwareMap);
    robot.mechanisms.ringCount = 3;
    robot.mechanisms.setCurrentPosition(startX, startY, robot.getTheta());

    //Waits for Start:
    waitForStart();

    /* Wobble Goal and Park */

    //Moves and Drops Wobble:
    robot.mechanisms.arm = 1;
    robot.mechanisms.runToPosition("forward", 1, 652.0, 150.0, robot.mainPower);
    robot.mechanisms.automateArm(robot.fastPower);

    //Parks and Resets:
    robot.mechanisms.arm = 0;
    robot.mechanisms.runToPosition("backward", 1, 652.0, 360.0, robot.mainPower);
    robot.mechanisms.automateArm(robot.fastPower);

    /* Stop */

    //Status Update:
    telemetry.addData("Status", "Stopped");
    telemetry.update();
  }
}
