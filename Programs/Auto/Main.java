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
  private double RPM = 3820.0;

  //Positioning Variables:
  private int position = 0;
  private boolean camera = true;

  /* OPMODE METHODS */

  @Override
  public void runOpMode() {
    /* Initialization */

    //Hardware Initialization:
    robot.init(hardwareMap, camera);
    robot.mechanisms.initMechanisms(hardwareMap);
    robot.mechanisms.setCurrentPosition(startX, startY, robot.getTheta());
    robot.mechanisms.automateClaw();

    //Waits for Start:
    waitForStart();

    /* Detection */

    //Gets the Ring Position and Setup:
    robot.mechanisms.automateFlywheel(robot.mechanisms.mainRPM);
    position = robot.getPixelsPosition();

    //Moves to Shooting Position:
    robot.mechanisms.runToPosition(560.0, 415.0, 1, robot.firePower, true);
    robot.mechanisms.turnGyro(-93.0, robot.firePower, true);

    /* Shooting */

    //Shoots Rings:
    robot.mechanisms.automateShooter(false);
    robot.mechanisms.automateShooter(true);
    robot.mechanisms.automateShooter(true);
    robot.mechanisms.automateFlywheel(robot.mechanisms.mainRPM);

    /* Wobble Goal Drop, Shooting, Park */

    //Checks the Case:
    if (position == 1) {
      //Drops Wobble:
      robot.mechanisms.automateClaw();
      robot.mechanisms.turnGyro(13.0, robot.firePower, true);
      robot.mechanisms.runToPosition(660.0, 380.0, 1, robot.firePower, true);
      robot.mechanisms.automateArm();

      //Turns and Moves to Second Wobble:
      robot.mechanisms.turnGyro(-96.0, robot.firePower, true);
      robot.mechanisms.automateArm();
      robot.mechanisms.runToPosition(640.0, 610.0, 1, robot.firePower, true);

      //Moves and Turns Back to Drop and Park:
      robot.mechanisms.automateClaw();
      robot.mechanisms.turnGyro(154.0, robot.firePower, true);
      robot.mechanisms.runToPosition(640.0, 400.0, 1, robot.firePower, true);
    }

    else if (position == 2) {
      //Drops Wobble:
      robot.mechanisms.automateClaw();
      robot.mechanisms.turnGyro(63.0, robot.firePower, true);
      robot.mechanisms.runToPosition(600.0, 360.0, 1, robot.firePower, true);
      robot.mechanisms.automateArm();

      //Turns to Second Wobble:
      robot.mechanisms.turnGyro(-138.0, robot.firePower, true);
      robot.mechanisms.automateArm();
      robot.mechanisms.automateIntake();

      //Moves to Second Wobble Goal:
      robot.mechanisms.automateFlywheel(RPM);
      robot.mechanisms.runToPosition(640.0, 640.0, 1, robot.firePower, true);
      robot.mechanisms.automateClaw();

      //Turns and Shoots Ring:
      robot.mechanisms.turnGyro(89.0, robot.firePower, true);
      robot.mechanisms.automateMagazine();
      robot.mechanisms.automateShooter(true);
      robot.mechanisms.automateShooter(true);

      //Moves to Drop Second Wobble Goal and Park:
      robot.mechanisms.automateIntake();
      robot.mechanisms.turnGyro(77.0, robot.firePower, true);
      robot.mechanisms.runToPosition(600.0, 330.0, 1, robot.firePower, true);
    }

    else {
      //Drops Wobble:
      robot.mechanisms.automateClaw();
      robot.mechanisms.turnGyro(60.0, robot.firePower, true);
      robot.mechanisms.runToPosition(640.0, 180.0, 1, robot.firePower, true);

      //Turns and Moves to Second Wobble:
      robot.mechanisms.automateArm();
      robot.mechanisms.turnGyro(-147.0, robot.firePower, true);
      robot.mechanisms.automateArm();

      //Grabs and Turns to Drop:
      robot.mechanisms.runToPosition(640.0, 600.0, 1, robot.firePower, true);
      robot.mechanisms.automateClaw();
      robot.mechanisms.turnGyro(172.0, robot.firePower, true);
      robot.mechanisms.runToPosition(640.0, 160.0, 1, robot.firePower, true);
    }
  }
}