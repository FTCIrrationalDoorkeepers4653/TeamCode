package org.firstinspires.ftc.teamcode.Systems.Core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Positions extends LinearOpMode {
  /* POSITION VARIABLES */

  //Objects:
  private static Robot robot = new Robot();

  /* OPMODE METHODS */

  //RunOpMode Method:
  @Override
  public void runOpMode() {
    /* Accesses Hardware Functions */
  }

  /* POSITION METHODS */

  //Finds the Best Path to Coordinate (Uses AutonomousMapping Application Dimensions 760x760):
  public void turnToPosition(String movement, int index, double startCoordinates[], double endCoordinates[], double power) {
    //Gets the Triangle and Rotates:
    double triangle[] = robot.getTriangle(startCoordinates, endCoordinates);
    turnGyro(movement, triangle[index], power);
  }

  //Move to Position Method:
  public void runToPosition(String movement, int index, double startCoordinates[], double endCoordinates[], double power) {
    //Gets the Triangle and Runs:
    double triangle[] = robot.getTriangle(startCoordinates, endCoordinates);
    double rotations = robot.getConvertedRotations(triangle[index]);
    runGyro(movement, rotations, power);
  }

  //Shift Position Method:
  public void shiftToPosition(String movement, int index, double startCoordinates[], double endCoordinates[], double power) {
    //Gets the Triangle and Shifts:
    double triangle[] = robot.getTriangle(startCoordinates, endCoordinates);
    double rotations = robot.getConvertedRotations(triangle[index]);
    shiftGyro(movement, rotations, power);
  }

  /* GYRO CORRECTION METHODS */

  //Makes turns with Gyro Corrections:
  public void turnGyro(String movement, double angle, double power) {
    //Calculates Turn:
    double gyroReset = robot.resetGyroValue();
    double turnValue = Math.abs(robot.calculateTurnRotations(angle));
    int timeRequired = Math.abs(robot.calculateTime(turnValue, power));

    //Turns Robot:
    robot.turnRobot(movement, Math.abs(turnValue), power);
    idle();
    sleep(timeRequired);
    robot.finishRun();
    idle();

    //Gyro Corrects Robot:
    gyroCorrect(angle, gyroReset, power);
  }

  //Runs with Gyro Corrects:
  public void runGyro(String movement, double rotations, double power) {
    //Calculates Resets Gyro Value:
    double gyroReset = robot.resetGyroValue();
    int millis = Math.abs(robot.calculateTime(rotations, power));

    //Runs the Robot Towards Position:
    robot.runRobot(movement, Math.abs(rotations), power);
    idle();
    sleep(millis);
    robot.finishRun();
    idle();

    //Gyro Corrects:
    gyroCorrect(0, gyroReset, power);
  }

  //Shifts with Gyro Corrects:
  public void shiftGyro(String movement, double rotations, double power) {
    //Calculates Resets Gyro Value:
    double gyroReset = robot.resetGyroValue();
    int millis = Math.abs(robot.calculateTime(rotations, power));

    //Shifts the Robot Towards Position:
    robot.shiftRobot(movement, Math.abs(rotations), power);
    idle();
    sleep(millis);
    robot.finishRun();
    idle();

    //Gyro Corrects:
    gyroCorrect(0, gyroReset, power);
  }

  //Gyro Correction Method (Reset Gyro Before):
  public void gyroCorrect(double expectedAngle, double resetValue, double power) {
    //Gyro Correction Calculation:
    String movement = "";
    double gyroValues[] = robot.getGyroCorrection(expectedAngle, resetValue);
    int millis = Math.abs(robot.calculateTime(gyroValues[1], power));

    //Checks the Case:
    if (gyroValues[0] == -1) {
      //Sets the Movement:
      movement = "left";
    }

    else if (gyroValues[0] == 1) {
      //Sets the Movement:
      movement = "right";
    }

    //Makes Correction:
    robot.turnRobot(movement, Math.abs(gyroValues[1]), power);
    idle();
    sleep(millis);
    robot.finishRun();
    idle();
  }
}
