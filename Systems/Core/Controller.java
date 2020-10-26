package org.firstinspires.ftc.teamcode.Systems.Core;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class Controller extends Positions {
  /* CONTROLLER VARIABLES */

  //Controller Objects:
  private static Robot robot = new Robot();
  private static ElapsedTime time = new ElapsedTime();

  //Constructor:
  public Controller() {
    super();
  }

  /* CONTROLLER CONTROL METHODS */

  //Controller Power Method:
  public static void applyControlPower(double power) {
    //Main Error Variables:
    int initialDifference = Math.abs(Math.abs(robot.leftFrontMotor.getTargetPosition())
      - Math.abs(robot.leftFrontMotor.getCurrentPosition()));
    double error = controlError(robot.leftFrontMotor.getCurrentPosition(),
      robot.leftFrontMotor.getTargetPosition(), initialDifference);
    robot.applyAllPowers(power);
  }

  //Controller Arm Power Method:
  public static void applyControlArmPower(double power) {
    int initialDifference = Math.abs(Math.abs(robot.mechanisms.baseArmMotor.getTargetPosition())
      - Math.abs(robot.mechanisms.baseArmMotor.getCurrentPosition()));
    double error = controlError(robot.mechanisms.baseArmMotor.getCurrentPosition(),
      robot.mechanisms.baseArmMotor.getTargetPosition(), initialDifference);
    robot.mechanisms.baseArmMotor.setPower(power);
  }

  /* CONTROL ALGORITHM METHODS */

  //Controller Turning Method:
  public static void applyControlTurn(int startCoordinates[], int middleCoordinates[],
    int endCoordinates[], double firstTurn, double secondTurn) {

  }

  //Controller Error Method:
  public static double controlError(int current, int target, int initialDifference) {
    //Calculates and Returns Error:
    double error = Math.abs(Math.abs(target) - Math.abs(current));
    error /= Math.abs(initialDifference);
    return error;
  }
}