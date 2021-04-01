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
  private double RPM = 3930.0;

  //Positioning Variables:
  private int position = 0;
  private int values[] = {1, 0, 0, 0, 0};

  //Setup Variables:
  private boolean auto = true;
  private boolean camera = true;

  /* OPMODE METHODS */

  @Override
  public void runOpMode() {
    /* Initialization */

    //Hardware Initialization:
    robot.init(hardwareMap, auto, camera);
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
    robot.mechanisms.automateFlywheel(auto);

    /* Wobble Goal Drop, Shooting, Park */

    //Checks the Case:
    if (position == 1) {
      //Drops Wobble:
      robot.mechanisms.turnGyro(5.0, robot.firePower, true);
      robot.mechanisms.runToPosition(660.0, 380.0, 1, robot.firePower, true);
      robot.mechanisms.automateClaw(!auto);
      robot.mechanisms.automateArm();

      //Turns and Moves to Second Wobble:
      robot.mechanisms.turnGyro(-96.0, robot.firePower, true);
      robot.mechanisms.automateArm();
      robot.mechanisms.runToPosition(640.0, 610.0, 1, robot.firePower, true);
      robot.mechanisms.automateClaw(!auto);

      //Moves and Turns Back to Drop and Park:
      robot.mechanisms.turnGyro(153.0, robot.firePower, true);
      robot.mechanisms.runToPosition(640.0, 400.0, 1, robot.firePower, true);
    }

    else if (position == 2) {
      //Drops Wobble:
      robot.mechanisms.turnGyro(55.0, robot.firePower, true);
      robot.mechanisms.runToPosition(600.0, 360.0, 1, robot.firePower, true);
      robot.mechanisms.automateClaw(!auto);
      robot.mechanisms.automateArm();

      //Turns to Second Wobble:
      robot.mechanisms.turnGyro(-138.0, robot.firePower, true);
      robot.mechanisms.automateArm();
      robot.mechanisms.automateIntake(robot.firePower);

      //Moves to Second Wobble Goal:
      robot.mechanisms.automateCustomFlywheel(RPM);
      robot.mechanisms.runToPosition(640.0, 640.0, 1, robot.firePower, true);
      robot.mechanisms.automateClaw(!auto);

      //Turns and Shoots Ring:
      robot.mechanisms.turnGyro(94.0, robot.firePower, true);
      sleep(robot.mechanisms.shooterWait);
      robot.mechanisms.automateShooter(0);
      robot.mechanisms.automateShooter(500);

      //Moves to Drop Second Wobble Goal and Park:
      robot.mechanisms.turnGyro(72.0, robot.firePower, true);
      robot.mechanisms.runToPosition(600.0, 340.0, 1, robot.firePower, true);
    }

    else {
      //Drops Wobble:
      robot.mechanisms.turnGyro(52.0, robot.uncoPower, true);
      robot.mechanisms.runToPosition(640.0, 180.0, 1, robot.uncoPower, true);
      robot.mechanisms.automateClaw(!auto);
      robot.mechanisms.automateArm();

      //Turns and Moves to Second Wobble:
      robot.mechanisms.turnGyro(-147.0, robot.uncoPower, true);
      robot.mechanisms.automateArm();
      robot.mechanisms.runToPosition(640.0, 600.0, 1, robot.uncoPower, true);
      robot.mechanisms.automateClaw(!auto);

      //Moves, Drops, Parks:
      robot.mechanisms.turnGyro(168.0, robot.uncoPower, true);
      robot.mechanisms.runToPosition(640.0, 160.0, 1, robot.uncoPower, true);
      robot.mechanisms.automateClaw(!auto);
      robot.mechanisms.runToPosition(640.0, 320.0, -1, robot.uncoPower, true);
    }
  }
}