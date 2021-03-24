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

  //Flywheel Variables:
  private double RPM = 3930.0;
  private double adjustedRPM = 3900.0;

  //Positioning Variables:
  private int position = 0;
  private int values[] = {1, 0, 0, 0, 0, 0};

  //Setup Variables:
  private boolean auto = true;
  private boolean teleOp = false;

  /* OPMODE METHODS */

  @Override
  public void runOpMode() {
    /* Initialization */

    //Hardware Initialization:
    robot.init(hardwareMap, true, true);
    robot.mechanisms.initMechanisms(hardwareMap, auto);
    robot.mechanisms.initCustomValues(values);
    robot.mechanisms.setCurrentPosition(startX, startY, robot.getTheta());

    //Waits for Start:
    waitForStart();

    /* Detection */

    //Gets the Ring Position and Setup:
    robot.mechanisms.automateFlywheel(auto);
    position = robot.getPixelsPosition();

    //Moves to Shooting Position:
    robot.mechanisms.runToPosition(560.0, 410.0, 1, robot.firePower, true);
    robot.mechanisms.turnGyro(-85.0, robot.firePower, true);

    /* Shooting */

    //Shoots Rings:
    robot.mechanisms.automateShooter(0);
    robot.mechanisms.automateShooter(robot.mechanisms.shooterWait);
    robot.mechanisms.automateShooter(robot.mechanisms.shooterWait);

    //Resets:
    robot.mechanisms.automateFlywheel(auto);
    robot.mechanisms.automatePark();

    /* Wobble Goal Drop, Shooting, Park */

    //Checks the Case:
    if (position == 1) {
      //Drops Wobble:
      robot.mechanisms.turnGyro(5.0, robot.firePower, true);
      robot.mechanisms.runToPosition(660.0, 380.0, 1, robot.firePower, true);
      robot.mechanisms.automateClaw(teleOp);
      robot.mechanisms.automateArm();

      //Turns and Moves to Second Wobble:
      robot.mechanisms.turnGyro(-96.0, robot.firePower, true);
      robot.mechanisms.automateArm();
      robot.mechanisms.runToPosition(640.0, 610.0, 1, robot.firePower, true);
      robot.mechanisms.automateClaw(teleOp);

      //Moves and Turns Back to Drop and Park:
      robot.mechanisms.turnGyro(153.0, robot.firePower, true);
      robot.mechanisms.runToPosition(640.0, 400.0, 1, robot.firePower, true);
    }

    else if (position == 2) {
      //Drops Wobble:
      robot.mechanisms.turnGyro(55.0, robot.firePower, true);
      robot.mechanisms.runToPosition(600.0, 360.0, 1, robot.firePower, true);
      robot.mechanisms.automateClaw(teleOp);
      robot.mechanisms.automateArm();

      //Turns to Second Wobble:
      robot.mechanisms.turnGyro(-138.0, robot.firePower, true);
      robot.mechanisms.automateArm();
      robot.mechanisms.automateIntake();

      //Moves to Second Wobble Goal:
      robot.mechanisms.automateCustomFlywheel(RPM);
      robot.mechanisms.runToPosition(640.0, 640.0, 1, robot.firePower, true);
      robot.mechanisms.automateClaw(teleOp);

      //Turns and Shoots Ring:
      robot.mechanisms.turnGyro(94.0, robot.firePower, true);
      sleep(robot.mechanisms.shooterWait);
      robot.mechanisms.automateShooter(0);
      robot.mechanisms.turnGyro(73.0, robot.firePower, true);

      //Moves to Drop Second Wobble Goal and Park:
      robot.mechanisms.automateCustomFlywheel(RPM);
      robot.mechanisms.automateIntake();
      robot.mechanisms.runToPosition(600.0, 340.0, 1, robot.firePower, true);
    }

    else {
      //Drops Wobble:
      robot.mechanisms.turnGyro(52.0, robot.firePower, true);
      robot.mechanisms.runToPosition(640.0, 180.0, 1, robot.firePower, true);
      robot.mechanisms.automateClaw(teleOp);
      robot.mechanisms.automateArm();

      //Turns and Moves to Starter Stack:
      robot.mechanisms.turnGyro(-153.0, robot.firePower, true);
      robot.mechanisms.automateCustomFlywheel(adjustedRPM);
      robot.mechanisms.shooter = 0;
      robot.mechanisms.runToPosition(517.0, 437.0, 1, robot.firePower, true);

      //Aims, Shoots Ring:
      robot.mechanisms.automateIntake();
      robot.mechanisms.runToPosition(517.0, 557.0, 1, robot.firePower, true);
      robot.mechanisms.turnGyro(109.0, robot.firePower, true);

      //Shoots and Waits:
      robot.mechanisms.automateShooter(0);
      sleep(robot.mechanisms.shooterWait);
      robot.mechanisms.automateFlywheel(teleOp);

      //Shoots and Parks:
      robot.mechanisms.automateShooter(robot.mechanisms.shooterWait);
      robot.mechanisms.shiftToPosition(517.0, 320.0, 1, robot.uncoPower, false);
      robot.mechanisms.automateArm();
    }
  }
}