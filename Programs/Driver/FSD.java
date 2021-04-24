package org.firstinspires.ftc.teamcode.Programs.Driver;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Systems.Core.Robot;

import lib.Capture;

@TeleOp(name="FSD")
public class FSD extends LinearOpMode {
  /* FSD VARIABLES */

  //Movement Variables:
  private Robot robot = new Robot();
  private boolean camera = true;
  private double turnValue = 20.0;
  private int wait = (robot.mechanisms.shooterWait * 4);

  /* FSD RUN METHODS */

  @Override
  public void runOpMode() {
    //Initialize Robot:
    robot.init(hardwareMap, camera);
    robot.mechanisms.initMechanisms(hardwareMap);

    //Mechanisms Startup:
    robot.mechanisms.automateIntake();
    robot.mechanisms.automateFlywheel(robot.mechanisms.mainRPM);
    robot.mechanisms.automateClaw();

    //Waits for Start:
    waitForStart();

    //Loops:
    mainLoop: while (opModeIsActive()) {
      //FSD:
      double objects[] = robot.getObjects();
      FSD(objects);
    }
  }

  //FSD Method:
  public void FSD(double objects[]) {
    //Checks the Case:
    if (objects[2] == 0) {
      //Turns to See Objects:
      robot.mechanisms.turnGyro(turnValue, robot.firePower, true);
    }

    else {
      //Gets the Motion Values:
      double inchesToCoordinates = (objects[0] / robot.POSITION_RATIO);
      double turnAngle = robot.calculateTurnAngle(objects[1]);

      //Runs Robot to Intake:
      robot.mechanisms.turnGyro(turnAngle, robot.gyroPower, true);
      robot.mechanisms.runToPosition(0, inchesToCoordinates, 1, robot.gyroPower, true);
      sleep(wait);

      //Runs Robot Back to Shoot:
      robot.mechanisms.runToPosition(0, 0, -1, robot.gyroPower, true);
      robot.mechanisms.turnGyro(-robot.getTheta(), robot.gyroPower, true);
      robot.mechanisms.automateShooter(false);
    }
  }
}