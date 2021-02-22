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
  private int values[] = {1, 0, 0, 0, 0};

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
    robot.mechanisms.initCustomValues(values);
    robot.mechanisms.setCurrentPosition(startX, startY, robot.getTheta());

    //Waits for Start:
    waitForStart();

    /* Detection */

    //Gets the Ring Position and Setup:
    robot.mechanisms.automateFlywheel(true);
    position = robot.getPixelsPosition();

    //Moves to Shooting Position:
    robot.mechanisms.runToPosition(560.0, 410.0, robot.firePower, 0.2, true);
    robot.mechanisms.turnGyro(-85.0, robot.firePower, 0.2, true);

    /* Shooting */

    //Shoots Rings:
    robot.mechanisms.automateShooter(100);
    robot.mechanisms.automateShooter(600);
    robot.mechanisms.automateShooter(600);
    robot.mechanisms.automateFlywheel(true);

    /* Wobble Goal Drop */

    //Checks the Case:
    if (position == 1) {
      //Drops Wobble:
      robot.mechanisms.turnGyro(5.0, robot.firePower, 0.2, true);
      robot.mechanisms.runToPosition(660.0, 380.0, robot.firePower, 0.2, true);
      robot.mechanisms.automateClaw(false);
      robot.mechanisms.automateArm();

      //Turns and Moves to Second Wobble:
      robot.mechanisms.turnGyro(-95.0, robot.firePower, 0.2, true);
      robot.mechanisms.automateArm();
      robot.mechanisms.runToPosition(640.0, 610.0, robot.firePower, 0.2, true);
      robot.mechanisms.automateClaw(false);

      //Moves and Turns Back to Drop:
      robot.mechanisms.turnGyro(152.0, robot.firePower, 0.2, true);
      robot.mechanisms.runToPosition(640.0, 400.0, robot.firePower, 0.2, true);
      robot.mechanisms.automateClaw(false);
    }

    else if (position == 2) {
      //Drops Wobble:
      robot.mechanisms.turnGyro(55.0, robot.firePower, 0.2, true);
      robot.mechanisms.runToPosition(600.0, 360.0, robot.firePower, 0.2, true);
      robot.mechanisms.automateClaw(false);
      robot.mechanisms.automateArm();

      //Turns and Moves to Second Wobble:
      robot.mechanisms.turnGyro(-137.0, robot.firePower, 0.2, true);
      robot.mechanisms.automateArm();
      robot.mechanisms.runToPosition(640.0, 640.0, robot.firePower, 0.2, true);
      robot.mechanisms.automateClaw(false);

      //Moves and Turns Back to Drop:
      robot.mechanisms.turnGyro(166.0, robot.firePower, 0.2, true);
      robot.mechanisms.runToPosition(600.0, 320.0, robot.firePower, 0.2, true);
      robot.mechanisms.automateClaw(false);
    }

    else {
      //Drops Wobble:
      robot.mechanisms.turnGyro(52.0, robot.firePower, 0.2, true);
      robot.mechanisms.runToPosition(640.0, 160.0, robot.uncoPower, 0.4, true);
      robot.mechanisms.automateClaw(false);
      robot.mechanisms.automateArm();

      //Turns and Moves to Second Wobble:
      robot.mechanisms.turnGyro(-148.0, robot.firePower, 0.2, true);
      robot.mechanisms.automateArm();
      robot.mechanisms.runToPosition(640.0, 600.0, robot.uncoPower, 0.4, true);
      robot.mechanisms.automateClaw(false);

      //Moves, Drops, Parks:
      robot.mechanisms.turnGyro(166.0, robot.firePower, 0.2, true);
      robot.mechanisms.runToPosition(640.0, 120.0, robot.uncoPower, 0.4, true);
      robot.mechanisms.automateClaw(false);
    }

    /* Stop */

    //Status Update:
    telemetry.addData("Status", "Stopped");
    telemetry.update();
  }
}