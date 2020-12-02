package org.firstinspires.ftc.teamcode.Programs.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Systems.Core.Robot;

@Autonomous(name="Main")
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
    robot.mechanisms.setCurrentPosition(startX, startY, robot.getTheta());

    //Waits for Start:
    waitForStart();

    /* Detection */

    //Gets the Ring Position:
    position = robot.getPixelsPosition();
    robot.mechanisms.shooter = 1;
    robot.mechanisms.operateFlywheel();

    //Moves to Shooting Position:
    robot.mechanisms.runToPosition(560.0, 420.0, robot.fastPower, true);
    robot.mechanisms.turnGyro(77.0, robot.mainPower, true);

    /* Shooting */

    //Waits and Shoots:
    robot.mechanisms.automateShooter();
    robot.mechanisms.completeCycle(robot.mechanisms.shooterRevWait);

    //Waits and Shoots:
    robot.mechanisms.automateShooter();
    robot.mechanisms.completeCycle(robot.mechanisms.shooterRevWait);
    robot.mechanisms.automateShooter();

    //Turns off Flywheel:
    robot.mechanisms.shooter = 0;
    robot.mechanisms.operateFlywheel();

    /* Wobble Goal Drop */

    //Checks the Case:
    if (position == 1) {
      //Turns and Drops Wobble:
      robot.mechanisms.arm = 1;
      robot.mechanisms.turnGyro(-10.0, robot.mainPower, true);
      robot.mechanisms.runToPosition(680.0, 380.0, robot.fastPower, false);
      robot.mechanisms.automateArm(robot.mainPower);
    }

    else if (position == 2) {
      //Turns and Drops Wobble:
      robot.mechanisms.arm = 1;
      robot.mechanisms.turnGyro(-60.0, robot.mainPower, true);
      robot.mechanisms.runToPosition(600.0, 320.0, robot.fastPower, false);
      robot.mechanisms.automateArm(robot.mainPower);
    }

    else {
      //Turns and Moves to Wobble Target:
      robot.mechanisms.arm = 1;
      robot.mechanisms.turnGyro(-50.0, robot.mainPower, true);
      robot.mechanisms.runToPosition(640.0, 140.0, robot.fastPower, false);

      //Drops Wobble and Parks:
      robot.mechanisms.automateArm(robot.mainPower);
      robot.mechanisms.runToPosition(550.0, 310.0, -robot.fastPower, false);
    }

    /* Stop */

    //Status Update:
    telemetry.addData("Status", "Stopped");
    telemetry.addData("Position", position);
    telemetry.update();
  }
}