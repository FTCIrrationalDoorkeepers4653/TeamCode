package org.firstinspires.ftc.teamcode.Systems.Core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Positions extends LinearOpMode {
  /* POSITION SETUP */

  //Objects:
  static Robot robot = new Robot();

  @Override
  public void runOpMode() {
    /* Accesses Hardware Functions */
  }

  /* POSITION MOVEMENT METHODS */

  //Finds the Best Path to Coordinate (Uses AutonomousMapping Application Dimensions 760x760):
  public void turnToPosition(String movement, int index, double startCoordinates[], double endCoordinates[], double power) {
    //Gets the Triangle and Rotates:
    double triangle[] = getTriangle(startCoordinates, endCoordinates);
    gyroCorrect(0, robot.resetGyroValue(), power);
    turnGyro(movement, triangle[index], power);
  }

  //Move to Position Method:
  public void runToPosition(String movement, int index, double startCoordinates[], double endCoordinates[], double power) {
    //Gets the Triangle and Runs:
    double triangle[] = getTriangle(startCoordinates, endCoordinates);
    double rotations = getConvertedRotations(triangle[index]);
    runGyro(movement, rotations, power);
  }

  //Shift Position Method:
  public void shiftToPosition(String movement, int index, double startCoordinates[], double endCoordinates[], double power) {
    //Gets the Triangle and Shifts:
    double triangle[] = getTriangle(startCoordinates, endCoordinates);
    double rotations = getConvertedRotations(triangle[index]);
    shiftGyro(movement, rotations, power);
  }

  /* POSITION CALCULATION METHODS */

  //Get Converted Rotations:
  public static double getConvertedRotations(double pixelDistance) {
    //Converts and Returns Data:
    double coordinatesToInches = (pixelDistance * robot.POSITION_RATIO);
    double inchesToRotations = (coordinatesToInches / robot.wheelCirc);
    return inchesToRotations;
  }

  //Gets Correct Gyro Correction Angle:
  public static double[] getTriangle(double startCoordinates[], double endCoordinates[]) {
    //Main Array:
    double triangle[] = new double[4];

    //Defines the Dimensions of Triangle:
    double x = endCoordinates[0] - startCoordinates[0];
    double y = endCoordinates[1] - startCoordinates[1];
    double h = Math.sqrt((x * x) + (y * y));
    double theta = robot.convertAngle(Math.abs(Math.atan2(x, y)), true);

    //Sets the Values (x, y, h, theta):
    triangle[0] = x;
    triangle[1] = y;
    triangle[2] = h;
    triangle[3] = theta;

    //Returns the Array:
    return triangle;
  }

  /* POSITION MOVEMENT METHODS */

  //Completes an Autonomous Cycle:
  public void completeCycle() {
    //Finishes the Run:
    idle();
    sleep(robot.delay);
    robot.finishRun();
    robot.mechanisms.mechanismsFinishRun();
    idle();
  }

  //Makes turns with Gyro Corrections:
  public void turnGyro(String movement, double angle, double power) {
    //Calculates Turn:
    double gyroReset = robot.resetGyroValue();
    double turnValue = Math.abs(robot.calculateTurnRotations(angle));

    //Moves Robot:
    robot.turnRobot(movement, Math.abs(turnValue), power);
    completeCycle();
    gyroCorrect(angle, gyroReset, power);
  }

  //Runs with Gyro Corrects:
  public void runGyro(String movement, double rotations, double power) {
    //Calculates Distance:
    double gyroReset = robot.resetGyroValue();

    //Moves Robot:
    robot.runRobot(movement, Math.abs(rotations), power);
    completeCycle();
    gyroCorrect(0, gyroReset, power);
  }

  //Shifts with Gyro Corrects:
  public void shiftGyro(String movement, double rotations, double power) {
    //Calculate Distance:
    double gyroReset = robot.resetGyroValue();

    //Moves Robot:
    robot.shiftRobot(movement, Math.abs(rotations), power);
    completeCycle();
    gyroCorrect(0, gyroReset, power);
  }

  //Gyro Correction Method (Reset Gyro Before):
  public void gyroCorrect(double expectedAngle, double resetValue, double power) {
    //Gyro Correction Calculation:
    String movement = "";
    double gyroValues[] = robot.getGyroCorrection(expectedAngle, resetValue);

    //Checks the Case:
    if (gyroValues[0] == -1) {
      //Sets the Movement:
      movement = "left";
      robot.turnRobot(movement, Math.abs(gyroValues[1]), power);
      completeCycle();
    }

    else if (gyroValues[0] == 1) {
      //Sets the Movement:
      movement = "right";
      robot.turnRobot(movement, Math.abs(gyroValues[1]), power);
      completeCycle();
    }
  }
}
