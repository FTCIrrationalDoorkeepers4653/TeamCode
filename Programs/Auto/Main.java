package org.firstinspires.ftc.teamcode.Programs.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Systems.Core.Robot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Main")
public class Main extends LinearOpMode {
  /* MAIN AUTO VARIABLES */

  //Movement Variables:
  private Robot robot = new Robot();
  private double startX = 560.0;
  private double startY = 760.0;
  private int position = 0;

  @Override
  public void runOpMode() {
    /* Initialization */

    //Status Updates:
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    //Hardware Initialization:
    robot.init(hardwareMap, true, true);
    robot.mechanisms.initMechanisms(hardwareMap);
    robot.mechanisms.ringCount = 3;
    robot.mechanisms.setCurrentPosition(startX, startY, robot.getTheta());

    //Waits for Start:
    waitForStart();

    /* Detection */

    //Gets the Ring Position:
    position = robot.getPixelsPosition();
    robot.mechanisms.automateFlywheel();

    //Moves to Shooting Position:
    robot.mechanisms.runToPosition("forward", 1, 560.0, 480.0, robot.fastPower);
    robot.mechanisms.turnGyro("right", 85.0, robot.mainPower);

    /* Shooting */

    //Moves Robot and Shoots:
    robot.mechanisms.runToPosition("backward", 2, 420.0, 480.0, robot.mainPower);
    robot.mechanisms.automateShooter();

    //Moves Robot and Shoots:
    robot.mechanisms.runToPosition("forward", 2, 440.0, 480.0, robot.mainPower);
    robot.mechanisms.automateShooter();

    //Moves Robot and Shoots Final:
    robot.mechanisms.runToPosition("forward", 2, 460.0, 480.0, robot.mainPower);
    robot.mechanisms.automateShooter();

    /* Wobble Goal and Park */

    //Checks the Case:
    if (position == 1) {
      //Turns and Drops Wobble:
      robot.mechanisms.arm = 1;
      robot.mechanisms.turnGyro("left", 25.0, robot.mainPower);
      robot.mechanisms.runToPosition("forward", 2, 720.0, 380.0, robot.fastPower);
      robot.mechanisms.automateArm(robot.fastPower);
    }

    else if (position == 2) {
      //Turns and Drops Wobble:
      robot.mechanisms.arm = 1;
      robot.mechanisms.turnGyro("left", 65.0, robot.mainPower);
      robot.mechanisms.runToPosition("forward", 2, 600.0, 360.0, robot.fastPower);
      robot.mechanisms.automateArm(robot.fastPower);
    }

    else {
      //Turns and Moves to Wobble Target:
      robot.mechanisms.arm = 1;
      robot.mechanisms.turnGyro("left", 55.0, robot.mainPower);
      robot.mechanisms.runToPosition("forward", 2, 640.0, 160.0, robot.fastPower);

      //Drops Wobble and Parks:
      robot.mechanisms.automateArm(robot.fastPower);
      robot.mechanisms.runToPosition("backward", 2, 550.0, 330.0, robot.fastPower);
    }

    //Resets the Arm:
    robot.mechanisms.arm = 0;
    robot.mechanisms.automateArm(robot.fastPower);

    /* Stop */

    //Status Update:
    telemetry.addData("Status", "Stopped");
    telemetry.addData("Position", position);
    telemetry.update();
  }
}