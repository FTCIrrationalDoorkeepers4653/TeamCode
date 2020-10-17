package org.firstinspires.ftc.teamcode.Systems.Core;

import com.qualcomm.robotcore.util.ElapsedTime;

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
  public static void applyControlPower(int targetPosition, double errorOffset, double constant) {
    //Main Error Variable:
    int initialDifference = Math.abs(Math.abs(targetPosition) - Math.abs(robot.leftFrontMotor.getCurrentPosition()));
    double error = applyControlError(robot.leftFrontMotor.getCurrentPosition(), targetPosition, initialDifference);

    //Loops through Error:
    while (error > errorOffset) {
      //Applies the Power:
      error = applyControlError(robot.leftFrontMotor.getCurrentPosition(), targetPosition, initialDifference);
      robot.applyAllPowers(error * constant);
    }
  }

  //Controller Turning Method:
  public static void applyControlTurn(int startCoordinates[], int middleCoordinates[],
    int endCoordinates[], double firstTurn, double secondTurn) {

  }

  //Controller Error Method:
  public static double applyControlError(int current, int target, int initialDifference) {
    //Calculates and Returns Error:
    double error = Math.abs(Math.abs(target) - Math.abs(current));
    error /= Math.abs(initialDifference);
    return error;
  }
}