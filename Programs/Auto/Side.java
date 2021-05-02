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

    /* Wobble Goal Drop and Park */

    //Checks the Case:
    if (position == 1) {
      //Drops Wobble and Parks:
      robot.mechanisms.automateClaw();
      robot.mechanisms.turnGyro(13.0, robot.firePower, true);
      robot.mechanisms.runToPosition(660.0, 380.0, 1, robot.firePower, true);
      robot.mechanisms.automateArm();
    }

    else if (position == 2) {
      //Drops Wobble and Parks:
      robot.mechanisms.automateClaw();
      robot.mechanisms.turnGyro(63.0, robot.firePower, true);
      robot.mechanisms.runToPosition(600.0, 360.0, 1, robot.firePower, true);
      robot.mechanisms.automateArm();
    }

    else {
      //Drops Wobble and Parks:
      robot.mechanisms.automateClaw();
      robot.mechanisms.turnGyro(60.0, robot.firePower, true);
      robot.mechanisms.runToPosition(640.0, 180.0, 1, robot.firePower, true);
      robot.mechanisms.automateArm();
      robot.mechanisms.runToPosition(640.0, 340.0, -1, robot.firePower, true);
    }
  }
}