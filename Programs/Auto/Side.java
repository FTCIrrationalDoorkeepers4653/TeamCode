package org.firstinspires.ftc.teamcode.Programs.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Systems.Core.Robot;

@Autonomous(name="Side")
public class Side extends LinearOpMode {
  /* SIDE AUTO VARIABLES */

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
    robot.mechanisms.automateFlywheel(robot.mainPower);

    //Moves to Shooting Position:
    robot.mechanisms.runToPosition("forward", 1, 560.0, 480.0, robot.fastPower, false);
    robot.mechanisms.turnGyro("right", 80.0, robot.mainPower, true);

    /* Shooting */

    //Waits and Shoots:
    robot.mechanisms.automateShooter();
    robot.mechanisms.completeCycle(robot.mechanisms.shooterWait);

    //Waits and Shoots:
    robot.mechanisms.automateShooter();
    robot.mechanisms.completeCycle(robot.mechanisms.shooterWait);

    //Shoots and Powers Down:
    robot.mechanisms.automateShooter();
    robot.mechanisms.automateFlywheel(robot.mainPower);

    /* Wobble Goal and Side */

    //Checks the Case:
    if (position == 1) {
      //Turns and Drops Wobble:
      robot.mechanisms.arm = 1;
      robot.mechanisms.turnGyro("left", 25.0, robot.mainPower, true);
      robot.mechanisms.runToPosition("forward", 2, 680.0, 380.0, robot.fastPower, false);
      robot.mechanisms.automateArm(robot.fastPower);
    }

    else if (position == 2) {
      //Turns and Drops Wobble:
      robot.mechanisms.arm = 1;
      robot.mechanisms.turnGyro("left", 50.0, robot.mainPower, true);
      robot.mechanisms.runToPosition("forward", 2, 600.0, 320.0, robot.fastPower, false);
      robot.mechanisms.automateArm(robot.fastPower);
    }

    else {
      //Turns and Moves to Wobble Target:
      robot.mechanisms.arm = 1;
      robot.mechanisms.turnGyro("left", 60.0, robot.mainPower, true);
      robot.mechanisms.runToPosition("forward", 2, 640.0, 160.0, robot.fastPower, false);

      //Drops Wobble and Parks:
      robot.mechanisms.automateArm(robot.fastPower);
      robot.mechanisms.runToPosition("backward", 2, 550.0, 310.0, robot.fastPower, false);
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
