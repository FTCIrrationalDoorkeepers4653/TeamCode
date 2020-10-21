package org.firstinspires.ftc.teamcode.Systems.Core;

public class Controller extends Positions {
  /* CONTROLLER VARIABLES */

  //Controller Objects:
  static Robot robot = new Robot();
  static Mechanisms mechanisms = new Mechanisms();

  /* CONTROLLER CONTROL METHODS */

  //Constructor:
  public Controller() {
    super();
  }

  //Controller Power Method:
  public static void applyControlPower(double power) {
    //Main Error Variables:
    int initialDifference = Math.abs(Math.abs(robot.leftFrontMotor.getTargetPosition())
        - Math.abs(robot.leftFrontMotor.getCurrentPosition()));
    double error = controlError(robot.leftFrontMotor.getCurrentPosition(),
        robot.leftFrontMotor.getTargetPosition(), initialDifference);
    double previousError = 0.0;

    //Loops through Error:
    mainLoop: while (error > previousError) {
      //Applies the Controlled Power:
      error = controlError(robot.leftFrontMotor.getCurrentPosition(),
          robot.leftFrontMotor.getTargetPosition(), initialDifference);
      robot.applyAllPowers((error * power));
      previousError = error;
    }
  }

  //Controller Arm Power Method:
  public static void applyControlArmPower(double power) {

  }

  //Controller Turning Method:
  public static void applyControlTurn(int startCoordinates[], int middleCoordinates[],
    int endCoordinates[], double firstTurn, double secondTurn) {

  }

  /* CONTROL CALCULATION METHODS */

  //Controller Error Method:
  public static double controlError(int current, int target, int initialDifference) {
    //Calculates and Returns Error:
    double error = Math.abs(Math.abs(target) - Math.abs(current));
    error /= Math.abs(initialDifference);
    return error;
  }
}