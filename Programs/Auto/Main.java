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

  /* OPMODE METHODS */

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

    //Shoots and Shuts Off:
    robot.mechanisms.automateShooter();
    robot.mechanisms.automateShooter();
    robot.mechanisms.automateShooter();
    robot.mechanisms.automateFlywheel();

    /* Wobble Goal Drop */

    //Checks the Case:
    if (position == 1) {
      //Turns and Drops Wobble:
      robot.mechanisms.turnGyro(5.0, robot.fastPower, true);
      robot.mechanisms.runToPosition(680.0, 380.0, robot.firePower, false);
      robot.mechanisms.automateArm(robot.easyPower);

      //Turns and Moves to Second Wobble:
      robot.mechanisms.turnGyro(-93.0, robot.fastPower, true);
      robot.mechanisms.runToPosition(640.0, 600.0, robot.firePower, false);
      robot.mechanisms.automateClaw();

      //Moves and Turns Back to Drop:
      robot.mechanisms.runToPosition(680.0, 380.0, -robot.firePower, false);
      robot.mechanisms.turnGyro(93.0, robot.fastPower, true);
      robot.mechanisms.automateClaw();
    }

    else if (position == 2) {
      //Turns and Drops Wobble:
      robot.mechanisms.turnGyro(55.0, robot.fastPower, true);
      robot.mechanisms.runToPosition(600.0, 320.0, robot.firePower, false);
      robot.mechanisms.automateArm(robot.easyPower);

      //Turns and Moves to Second Wobble:
      robot.mechanisms.turnGyro(-133.0, robot.fastPower, true);
      robot.mechanisms.runToPosition(640.0, 600.0, robot.firePower, false);
      robot.mechanisms.automateClaw();

      //Moves and Turns Back to Drop:
      robot.mechanisms.runToPosition(600.0, 380.0, -robot.firePower, false);
      robot.mechanisms.turnGyro(133.0, robot.fastPower, true);
      robot.mechanisms.automateClaw();
    }

    else {
      //Turns and Moves to Wobble Target:
      robot.mechanisms.turnGyro(45.0, robot.fastPower, true);
      robot.mechanisms.runToPosition(640.0, 140.0, robot.firePower, false);
      robot.mechanisms.automateArm(robot.easyPower);

      //Turns and Moves to Second Wobble:
      robot.mechanisms.turnGyro(-143.0, robot.fastPower, true);
      robot.mechanisms.runToPosition(640.0, 600.0, robot.firePower, false);
      robot.mechanisms.automateClaw();

      //Moves, Drops, Parks:
      robot.mechanisms.runToPosition(640.0, 140.0, -robot.firePower, false);
      robot.mechanisms.turnGyro(143.0, robot.fastPower, true);
      robot.mechanisms.automateClaw();
      robot.mechanisms.runToPosition(550.0, 310.0, -robot.firePower, false);
    }

    /* Stop */

    //Status Update:
    telemetry.addData("Status", "Stopped");
    telemetry.addData("Position", position);
    telemetry.update();
  }
}