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
    robot.mechanisms.automateFlywheel();

    //Moves to Shooting Position:
    robot.mechanisms.runToPosition(560.0, 410.0, robot.firePower, true);
    robot.mechanisms.turnGyro(-81.0, robot.fastPower, true);

    /* Shooting */

    //Waits and Shoots:
    robot.mechanisms.automateShooter();
    robot.mechanisms.completeCycle(robot.mechanisms.shooterRevWait);
    robot.mechanisms.automateShooter();

    //Waits and Shoots:
    robot.mechanisms.completeCycle(robot.mechanisms.shooterRevWait);
    robot.mechanisms.automateShooter();
    robot.mechanisms.automateFlywheel();

    /* Wobble Goal Drop */

    //Checks the Case:
    if (position == 1) {
      //Turns and Drops Wobble:
      robot.mechanisms.turnGyro(5.0, robot.fastPower, true);
      robot.mechanisms.runToPosition(680.0, 380.0, robot.firePower, false);
      robot.mechanisms.automateArm(robot.slowPower);

      //Turns and Moves to Second Wobble:
      robot.mechanisms.turnGyro(-95.0, robot.fastPower, true);
      robot.mechanisms.runToPosition(640.0, 640.0, robot.firePower, false);
      robot.mechanisms.automateClaw();
      robot.mechanisms.completeCycle(robot.mechanisms.shooterRevWait);
      robot.mechanisms.automateArm(robot.mainPower);

      //Moves and Turns Back to Drop:
      robot.mechanisms.runToPosition(680.0, 380.0, -robot.firePower, false);
      robot.mechanisms.automateClaw();
      robot.mechanisms.turnGyro(95.0, robot.fastPower, true);
      robot.mechanisms.automateArm(robot.slowPower);
      robot.mechanisms.automateArm(robot.mainPower);
    }

    else if (position == 2) {
      //Turns and Drops Wobble:
      robot.mechanisms.turnGyro(55.0, robot.fastPower, true);
      robot.mechanisms.runToPosition(600.0, 320.0, robot.firePower, false);
      robot.mechanisms.automateArm(robot.slowPower);

      //Turns and Moves to Second Wobble:
      robot.mechanisms.turnGyro(-145.0, robot.fastPower, true);
      robot.mechanisms.runToPosition(640.0, 640.0, robot.firePower, false);
      robot.mechanisms.automateClaw();
      robot.mechanisms.completeCycle(robot.mechanisms.shooterRevWait);
      robot.mechanisms.automateArm(robot.mainPower);

      //Moves and Turns Back to Drop:
      robot.mechanisms.runToPosition(600.0, 380.0, -robot.firePower, false);
      robot.mechanisms.automateClaw();
      robot.mechanisms.turnGyro(145.0, robot.fastPower, true);
      robot.mechanisms.automateArm(robot.slowPower);
      robot.mechanisms.automateArm(robot.mainPower);
    }

    else {
      //Turns and Moves to Wobble Target:
      robot.mechanisms.turnGyro(45.0, robot.fastPower, true);
      robot.mechanisms.runToPosition(640.0, 140.0, robot.firePower, false);
      robot.mechanisms.automateArm(robot.slowPower);

      //Turns and Moves to Second Wobble:
      robot.mechanisms.turnGyro(-135.0, robot.fastPower, true);
      robot.mechanisms.runToPosition(640.0, 640.0, robot.firePower, false);
      robot.mechanisms.automateClaw();
      robot.mechanisms.completeCycle(robot.mechanisms.shooterRevWait);
      robot.mechanisms.automateArm(robot.mainPower);

      //Moves, Drops, Parks:
      robot.mechanisms.runToPosition(640.0, 140.0, -robot.firePower, false);
      robot.mechanisms.automateClaw();
      robot.mechanisms.turnGyro(135.0, robot.fastPower, true);
      robot.mechanisms.automateArm(robot.slowPower);
      robot.mechanisms.runToPosition(550.0, 310.0, -robot.firePower, false);
      robot.mechanisms.automateArm(robot.mainPower);
    }

    /* Stop */

    //Status Update:
    telemetry.addData("Status", "Stopped");
    telemetry.addData("Position", position);
    telemetry.update();
  }
}