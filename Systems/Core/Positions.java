package org.firstinspires.ftc.teamcode.Systems.Core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Positions extends LinearOpMode {
  /* POSITION SETUP */

  //Position Variables:
  private static Robot robot = new Robot();
  private static double positionX = 0.0;
  private static double positionY = 0.0;
  private static double theta = 0.0;

  @Override
  public void runOpMode() {
    /* Accesses Hardware Functions */
  }

  /* POSITION CONTROL METHODS */

  //Set Current Position Method:
  public void setCurrentPosition(double x, double y, double angle) {
    //Sets the Current Position:
    positionX = x;
    positionY = y;
    theta = angle;
  }

  //Resets the Current Position Method:
  public void resetCurrentPosition() {
    //Sets the Current Position:
    positionX = 0.0;
    positionY = 0.0;
    theta = robot.getTheta();
  }

  /* POSITION MOVEMENT METHODS */

  //Turn to Position Method:
  public void turnToPosition(String movement, int index, double targetX, double targetY,
    double power, boolean correct) {
    //Gets the Triangle:
    double start[] = {positionX, positionY};
    double end [] = {targetX, targetY};
    double triangle[] = getTriangle(start, end);
    double fullAngle = (theta + triangle[index]);

    //Turns to Position:
    turnGyro(movement, fullAngle, power, correct);
    theta = robot.getTheta();
  }

  //Move to Position Method:
  public void runToPosition(String movement, int index, double targetX, double targetY,
    double power, boolean correct) {
    //Gets the Triangle:
    double start[] = {positionX, positionY};
    double end [] = {targetX, targetY};
    double triangle[] = getTriangle(start, end);

    //Runs to Position:
    double rotations = getConvertedRotations(triangle[index]);
    runGyro(movement, rotations, power, correct);

    //Sets the New Position:
    positionX = targetX;
    positionY = targetY;
    theta = robot.getTheta();
  }

  //Shift Position Method:
  public void shiftToPosition(String movement, int index, double targetX, double targetY,
    double power, boolean correct) {
    //Gets the Triangle:
    double start[] = {positionX, positionY};
    double end [] = {targetX, targetY};
    double triangle[] = getTriangle(start, end);

    //Shifts to Position:
    double rotations = getConvertedRotations(triangle[index]);
    shiftGyro(movement, rotations, power, correct);

    //Sets the New Position:
    positionX = targetX;
    positionY = targetY;
    theta = robot.getTheta();
  }

  /* POSITION CALCULATION METHODS */

  //Completes an Main Cycle:
  public void completeCycle(int timeMillis) {
    //Finishes the Run:
    idle();
    sleep(timeMillis);
    robot.finishRun();
    robot.mechanisms.mechanismsFinishRun();
    idle();
  }

  //Calculates Time Required To Complete Operation:
  public static int calculateTime(double rotations, double power) {
    //Calculates the Time and Returns:
    double timeInSeconds = Math.abs((rotations / (robot.wheelRPS * power)));
    int timeInMillis = (int) (timeInSeconds * 1000.0);
    return timeInMillis;
  }

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

    //Formats And Returns Triangle:
    triangle[0] = x;
    triangle[1] = y;
    triangle[2] = h;
    triangle[3] = theta;
    return triangle;
  }

  /* POSITION MOVEMENT METHODS */

  //Makes turns with Gyro Corrections:
  public void turnGyro(String movement, double angle, double power, boolean correct) {
    //Calculates Turn:
    double gyroReset = robot.resetGyroValue();
    double turnValue = Math.abs(robot.calculateTurnRotations(angle));
    int timeRequired = Math.abs(calculateTime(turnValue, power));

    //Turns Robot and Corrects:
    robot.turnRobot(movement, Math.abs(turnValue), power);
    completeCycle(timeRequired);

    //Checks the Case:
    if (correct) {
      //Corrects the Motion:
      gyroCorrect(angle, gyroReset, power);
    }

    //Sets the Current Theta:
    theta = robot.getTheta();
  }

  //Runs with Gyro Corrects:
  public void runGyro(String movement, double rotations, double power, boolean correct) {
    //Calculates Resets Gyro Value:
    double gyroReset = robot.resetGyroValue();
    int timeRequired = Math.abs(calculateTime(rotations, power));

    //Runs Robot and Corrects:
    robot.runRobot(movement, Math.abs(rotations), power);
    completeCycle(timeRequired);

    //Checks the Case:
    if (correct) {
      //Corrects the Motion:
      gyroCorrect(theta, gyroReset, power);
    }

    //Sets the Current Theta:
    theta = robot.getTheta();
  }

  //Shifts with Gyro Corrects:
  public void shiftGyro(String movement, double rotations, double power, boolean correct) {
    //Calculates Resets Gyro Value:
    double gyroReset = robot.resetGyroValue();
    int timeRequired = Math.abs(calculateTime(rotations, power));

    //Shifts Robot and Corrects:
    robot.shiftRobot(movement, Math.abs(rotations), power);
    completeCycle(timeRequired);

    //Checks the Case:
    if (correct) {
      //Corrects the Motion:
      gyroCorrect(theta, gyroReset, power);
    }

    //Sets the Current Theta:
    theta = robot.getTheta();
  }

  //Gyro Correction Method (Reset Gyro Before):
  public void gyroCorrect(double expectedAngle, double resetValue, double power) {
    //Gyro Correction Calculation:
    String movement = "";
    double gyroValues[] = robot.getGyroCorrection(expectedAngle, resetValue);
    int timeRequired = Math.abs(calculateTime(gyroValues[1], power));

    //Checks the Case:
    if (gyroValues[0] < 0) {
      //Sets the Movement:
      movement = "left";
    }

    else if (gyroValues[0] > 0) {
      //Sets the Movement:
      movement = "right";
    }

    //Checks the Case:
    if (robot.isWithinRange(gyroValues[1], robot.gyroStabilization)) {
      //Makes Correction:
      robot.turnRobot(movement, Math.abs(gyroValues[1]), power);
      completeCycle(timeRequired);
    }

    //Sets the Current Theta:
    theta = robot.getTheta();
  }
}