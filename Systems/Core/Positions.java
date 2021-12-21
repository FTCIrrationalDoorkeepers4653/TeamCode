package org.firstinspires.ftc.teamcode.Systems.Core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.ArrayList;

public class Positions extends LinearOpMode {
  /* POSITION SETUP VARIABLES */

  //Position Variables:
  private static Robot robot = new Robot();
  private static double positionX = 0.0;
  private static double positionY = 0.0;
  private static double theta = 0.0;

  //Position Block Variables:
  private static ArrayList<Double> roadblockX = new ArrayList<Double>();
  private static ArrayList<Double> roadblockY = new ArrayList<Double>();

  /* POSITION CONTROL METHODS */

  @Override
  public void runOpMode() {
    /* Access Hardware Methods */
  }

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

  //Sets the Roadblocks on the Field:
  public void setRoadblocks(double x[], double y[]) {
    //Checks the Case:
    if (x.length == y.length) {
      //Sets the Roadblocks:
      roadblockX = toDoubleList(x);
      roadblockY = toDoubleList(y);
    }
  }

  /* POSITION MOVEMENT METHODS */

  //Move to Position Method:
  public void runToPosition(double targetX, double targetY, double direction, double power,
    boolean correct) {
    //Gets the Triangle:
    double start[] = {positionX, positionY};
    double end [] = {targetX, targetY};
    double triangle[] = getTriangle(start, end);

    //Checks the Case:
    if (isRoadClear(targetX, targetY)) {
      //Runs to Position:
      double rotations = (direction * getConvertedRotations(triangle[2]));
      runGyro(rotations, power, correct);

      //Sets the New Position:
      positionX = targetX;
      positionY = targetY;
      theta = robot.getTheta();
    }
  }

  //Turn to Position Method:
  public void turnToPosition(double targetX, double targetY, double power, boolean correct) {
    //Gets the Triangle:
    double start[] = {positionX, positionY};
    double end [] = {targetX, targetY};
    double triangle[] = getTriangle(start, end);

    //Checks the Case:
    if (isRoadClear(targetX, targetY)) {
      //Turns and Sets New Position:
      double turnAngle = (-theta + triangle[3]);
      turnGyro(turnAngle, power, correct);
      theta = robot.getTheta();
    }
  }

  /* POSITION CORE METHODS */

  //Runs with Gyro Corrects:
  public void runGyro(double rotations, double power, boolean correct) {
    //Runs:
    robot.runRobot(rotations, power);

    //Checks the Case:
    if (correct) {
      //Corrects the Motion:
      gyroCorrect(theta);
    }

    //Sets the Current Theta:
    theta = robot.getTheta();
  }

  //Makes turns with Gyro Corrections:
  public void turnGyro(double angle, double power, boolean correct) {
    //Calculates and Turns:
    double turnValue = robot.calculateTurnRotations(angle);
    double expected = (theta + angle);
    robot.turnRobot(turnValue, power);

    //Checks the Case:
    if (correct) {
      //Corrects the Motion:
      gyroCorrect(expected);
    }

    //Sets the Current Theta:
    theta = robot.getTheta();
  }

  //Gyro Correction Method (Reset Gyro Before):
  public void gyroCorrect(double expectedAngle) {
    //Checks the Case:
    if (robot.isWithinRange(robot.getTheta(), expectedAngle, robot.gyroStabilization)) {
      //Gyro Correction:
      double gyroValue = robot.getGyroCorrection(expectedAngle);
      robot.turnRobot(gyroValue, robot.gyroPower);
    }

    //Sets the Current Theta:
    theta = robot.getTheta();
  }

  /* POSITION PATH METHODS */

  //Find Open Paths to Target Method:
  public static boolean isRoadClear(double targetX, double targetY) {
    //Main Variables:
    int turns = 0;
    int counter = 0;
    double mainLine[] = findLineEquation(positionX, positionY, targetX, targetY);

    //Loops through Array:
    mainLoop: while (turns < roadblockX.size()) {
      //Checks the Case:
      if (!isPointOnLine(mainLine, roadblockX.get(turns), roadblockY.get(turns))) {
        //Adds to the Counter:
        counter++;
      }

      turns++;
    }

    //Checks the Case:
    if (counter == roadblockX.size()) {
      //Returns the Value:
      return true;
    }

    else {
      //Returns the Value:
      return false;
    }
  }

  //Finds the Equation of Line:
  public static double[] findLineEquation(double x, double y, double targetX, double targetY) {
    //Finds the Initial Values:
    double slope = ((targetY - y) / (targetX - x));
    double intercept = (y - (slope * x));

    //Formats and Returns:
    double array[] = {slope, intercept};
    return array;
  }

  //Finds If Point on Line:
  public static boolean isPointOnLine(double line[], double x, double y) {
    //Gets the Line:
    double left = y;
    double right = ((x * line[0]) + line[1]);

    //Checks the Case:
    if (left == right) {
      //Returns the Value:
      return true;
    }

    else {
      //Returns the Value:
      return false;
    }
  }

  //Gets Position Triangle:
  public static double[] getTriangle(double startCoordinates[], double endCoordinates[]) {
    //Main Array Variable:
    double triangle[] = new double[6];

    //Defines the Dimensions of Triangle:
    double x = startCoordinates[0] - endCoordinates[0];
    double y = startCoordinates[1] - endCoordinates[1];
    double h = Math.sqrt((x * x) + (y * y));
    double xDirection = (x / Math.abs(x));
    double yDirection = (y / Math.abs(y));

    //Gets the Angle of Triangle:
    double angle = Math.abs(robot.convertAngle(Math.atan(x / y), true));
    angle = transformAngle(angle, x, y);

    //Formats Triangle:
    triangle[0] = x;
    triangle[1] = y;
    triangle[2] = h;
    triangle[3] = angle;
    triangle[4] = xDirection;
    triangle[5] = yDirection;

    //Returns the Triangle:
    return triangle;
  }

  /* POSITION UTILITY METHODS */

  //Gets the Transformed Angle:
  public static double transformAngle(double angle, double xDiff, double yDiff) {
    //Main Value:
    double localAngle = angle;

    //Checks the Case:
    if (xDiff > 0 && yDiff < 0 || xDiff < 0 && yDiff > 0) {
      //Sets the Angle:
      localAngle = -localAngle;
    }

    //Returns the Angle:
    return localAngle;
  }

  //Get Converted Rotations:
  public static double getConvertedRotations(double pixelDistance) {
    //Converts and Returns Data:
    double coordinatesToInches = (pixelDistance * robot.POSITION_RATIO);
    double inchesToRotations = (coordinatesToInches / robot.wheelCirc);
    return inchesToRotations;
  }

  //Array to ArrayList Method:
  public static ArrayList<Double> toDoubleList(double array[]) {
    //Main Variables:
    ArrayList<Double> list = new ArrayList<Double>();
    int turns = 0;

    //Loops through Array:
    mainLoop: while (turns < array.length) {
      //Adds to List:
      list.add(array[turns]);

      turns++;
    }

    //Returns List:
    return list;
  }
}