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
    robot.mechanisms.runToPosition(560.0, 410.0, robot.uncoPower, true);
    robot.mechanisms.turnGyro(-85.0, robot.mainPower, true);

    /* Shooting */

    //Shoots and Shuts Off:
    robot.mechanisms.automateShooter();
    robot.mechanisms.automateShooter();
    robot.mechanisms.automateShooter();
    robot.mechanisms.automateFlywheel();
    robot.mechanisms.automateClaw();

    /* Wobble Goal Drop */

    //Checks the Case:
    if (position == 1) {
      //Drops Wobble:
      robot.mechanisms.turnGyro(5.0, robot.firePower, true);
      robot.mechanisms.runToPosition(660.0, 380.0, robot.uncoPower, true);
      robot.mechanisms.automateClaw();
      robot.mechanisms.automateArm(robot.mainPower);

      //Turns and Moves to Second Wobble:
      robot.mechanisms.automateClaw();
      robot.mechanisms.turnGyro(-95.0, robot.firePower, true);
      robot.mechanisms.automateClaw();
      robot.mechanisms.runToPosition(640.0, 630.0, robot.uncoPower, true);
      robot.mechanisms.automateClaw();

      //Moves and Turns Back to Drop:
      robot.mechanisms.turnGyro(152.0, robot.firePower, true);
      robot.mechanisms.runToPosition(640.0, 400.0, robot.uncoPower, true);
    }

    else if (position == 2) {
      //Drops Wobble:
      robot.mechanisms.turnGyro(55.0, robot.firePower, true);
      robot.mechanisms.runToPosition(600.0, 360.0, robot.uncoPower, true);
      robot.mechanisms.automateClaw();
      robot.mechanisms.automateArm(robot.mainPower);

      //Turns and Moves to Second Wobble:
      robot.mechanisms.automateClaw();
      robot.mechanisms.turnGyro(-137.0, robot.firePower, true);
      robot.mechanisms.automateClaw();
      robot.mechanisms.runToPosition(640.0, 660.0, robot.uncoPower, true);
      robot.mechanisms.automateClaw();

      //Moves and Turns Back to Drop:
      robot.mechanisms.turnGyro(165.0, robot.firePower, true);
      robot.mechanisms.runToPosition(600.0, 320.0, robot.uncoPower, true);
    }

    else {
      //Drops Wobble and Parks:
      robot.mechanisms.turnGyro(50.0, robot.firePower, true);
      robot.mechanisms.runToPosition(640.0, 160.0, robot.uncoPower, true);
      robot.mechanisms.automateClaw();
      robot.mechanisms.automateArm(robot.mainPower);

      //Turns and Moves to Second Wobble:
      robot.mechanisms.automateClaw();
      robot.mechanisms.turnGyro(-147.0, robot.firePower, true);
      robot.mechanisms.automateClaw();
      robot.mechanisms.runToPosition(640.0, 600.0, robot.uncoPower, true);
      robot.mechanisms.automateClaw();

      //Moves, Drops, Parks:
      robot.mechanisms.turnGyro(168.0, robot.firePower, true);
      robot.mechanisms.runToPosition(640.0, 120.0, robot.uncoPower, true);
      robot.mechanisms.automateClaw();
      robot.mechanisms.runToPosition(640.0, 300.0, -robot.uncoPower, true);
    }

    /* Stop */

    //Status Update:
    telemetry.addData("Status", "Stopped");
    telemetry.addData("Position", position);
    telemetry.update();
  }
}