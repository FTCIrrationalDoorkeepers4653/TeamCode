package org.firstinspires.ftc.teamcode.Systems.Movement;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Positions extends LinearOpMode {
  /* POSITION VARIABLES */

  //Objects:
  static Robot robot = new Robot();

  //Movement Variables:
  static double movementOneRotations = 0, movementTwoRotations = 0;
  static double globalTheta = 0;

  /* OPMODE METHODS */

  //RunOpMode Method:
  @Override
  public void runOpMode() {
    /* Accesses Hardware Functions */
  }

  /* POSITION METHODS */

  //Finds the Best Path to Coordinate (Uses AutonomousMapping Application Dimensions 760x760):
  public static void findPath(String path[], double startCoordinates[], double endCoordinates[], boolean turnFirst) {
    //Defines the Dimensions of Triangle:
    double triangle[] = getTriangle(startCoordinates, endCoordinates);

    //Defines the Values based on the Path (Movement 1):
    if (path[0].equalsIgnoreCase("left") || path[0].equalsIgnoreCase("right")) {
      if (turnFirst) {
        //Calculates the Turn Angle:
        double thetaInRotations = robot.calculateTurnRotations(triangle[3]);
        movementOneRotations = Math.abs(thetaInRotations);

        //Setting Global Theta Value:
        if (path[0].equalsIgnoreCase("left")) {
          //Sets Value:
          globalTheta = triangle[3];
        }

        else if (path[0].equalsIgnoreCase("right")) {
          //Sets Value:
          globalTheta = -triangle[3];
        }
      }

      else { //Shifting First is Discouraged
        //Calculates the Shift Movement:
        double coordinatesToInches = (triangle[0] * robot.POSITION_RATIO);
        double inchesToRotations = (coordinatesToInches / robot.wheelCirc);
        movementOneRotations = Math.abs(inchesToRotations);
      }
    }

    else if (path[0].equalsIgnoreCase("forward") || path[0].equalsIgnoreCase("backward")) {
      //Calculates Forwards/Backwards Rotations:
      double coordinatesToInches = (triangle[1] * robot.POSITION_RATIO);
      double inchesToRotations = (coordinatesToInches / robot.wheelCirc);
      movementOneRotations = Math.abs(inchesToRotations);
    }

    //Defines the Values based on the Path (Movement 2):
    if (path[1].equalsIgnoreCase("left") || path[1].equalsIgnoreCase("right")) {
      //Calculates the Shift Movement:
      double coordinatesToInches = (triangle[0] * robot.POSITION_RATIO);
      double inchesToRotations = (coordinatesToInches / robot.wheelCirc);
      movementTwoRotations = Math.abs(inchesToRotations);
    }

    else if (path[1].equalsIgnoreCase("forward") || path[1].equalsIgnoreCase("backward")) {
      //Calculates Forwards/Backwards Rotations:
      double coordinatesToInches = (triangle[1] * robot.POSITION_RATIO);
      double inchesToRotations = (coordinatesToInches / robot.wheelCirc);
      movementTwoRotations = Math.abs(inchesToRotations);
    }
  }

  //Move to Position Method:
  public void moveToPosition(String path[], double power, boolean turnFirst) {
    //Movements One and Two:
    int realTimeMillisOne = calculateTime(movementOneRotations, power);
    int realTimeMillisTwo = calculateTime(movementTwoRotations, power);

    //Movement One Code:
    if (path[0].equalsIgnoreCase("left") || path[0].equalsIgnoreCase("right")) {
      //Checks the Case:
      if (turnFirst) {
        //Makes the Gyro Turn:
        turnGyro(path[0], globalTheta, power);
      }

      else {
        //Makes Shift Gyro (Discouraged):
        shiftGyro(path[0], movementOneRotations, power);
      }
    }

    else if (path[0].equalsIgnoreCase("forward") || path[0].equalsIgnoreCase("backward")) {
      //Turns the Robot Towards Position:
      robot.runRobot(path[0], movementOneRotations, power);
      idle();
      sleep(realTimeMillisOne);
      robot.finishRun();
      idle();
    }

    //Movement Two Code:
    if (path[1].equalsIgnoreCase("left") || path[1].equalsIgnoreCase("right")) {
      //Makes Shift Gyro:
      shiftGyro(path[1], movementTwoRotations, power);
    }

    else if (path[1].equalsIgnoreCase("forward") || path[1].equalsIgnoreCase("backward")) {
      //Runs the Robot Towards Position:
      robot.runRobot(path[1], movementTwoRotations, power);
      idle();
      sleep(realTimeMillisTwo);
      robot.finishRun();
      idle();
    }
  }

  /* CALCULATION METHODS */

  //Calculates Time Required To Complete Operation:
  public static int calculateTime(double rotations, double power) {
    //Calculates the Time Using Ratio:
    double timeInSeconds = Math.abs((rotations / (robot.wheelRPS * power)));
    int timeInMillis = (int)(timeInSeconds * 1000.0);

    //Returns the Value:
    return timeInMillis;
  }

  //Gets Correct Gyro Correction Angle:
  public static double[] getTriangle(double startCoordinates[], double endCoordinates[]) {
    //Main Array:
    double triangle[] = new double[4];

    //Defines the Dimensions of Triangle:
    double x = endCoordinates[0] - startCoordinates[0];
    double y = endCoordinates[1] - startCoordinates[1];
    double h = Math.sqrt(((x * x) + (y * y)));
    double theta = robot.convertAngle((Math.atan(y / x)), true);

    //Sets the Values (x, y, h, theta):
    triangle[0] = x;
    triangle[1] = y;
    triangle[2] = h;
    triangle[3] = theta;

    //Returns the Array:
    return triangle;
  }

  /* GYRO METHODS */

  //Gyro Correction Method (Reset Gyro Before):
  public void gyroCorrect(double expectedAngle, double resetValue, double power) {
    //Gyro Correction Calculation:
    double gyroValues[] = robot.getGyroCorrection(expectedAngle, resetValue);
    String turnValue = "";

    //Sets the turn Value:
    if (gyroValues[0] == -1) {
      turnValue = "right";
    }

    else if (gyroValues[0] == 1) {
      turnValue = "left";
    }

    //Calculates Time Required:
    int millis = calculateTime(gyroValues[1], power);

    //Makes Correction:
    robot.turnRobot(turnValue, gyroValues[1], power);
    idle();
    sleep(millis);
    robot.finishRun();
    idle();
  }

  //Makes turns with Gyro Corrections:
  public void turnGyro(String direction, double angle, double power) {
    //Calculates Turn:
    double gyroResetDeliver = robot.resetGyroValue();
    double turnValueDeliver = robot.calculateTurnRotations(angle);
    int timeRequiredDeliver = calculateTime(turnValueDeliver, power);

    //Turns Robot:
    robot.turnRobot(direction, turnValueDeliver, power);
    idle();
    sleep(timeRequiredDeliver);
    robot.finishRun();
    idle();

    //Gyro Corrects Robot:
    gyroCorrect(angle, gyroResetDeliver, power);
  }

  //Shifts with Gyro Corrects:
  public void shiftGyro(String direction, double rotations, double power) {
    //Calculates Resets Gyro Value:
    double gyroReset = robot.resetGyroValue();
    int millis = calculateTime(rotations, power);

    //Shifts the Robot Towards Position:
    robot.shiftRobot(direction, movementOneRotations, power);
    idle();
    sleep(millis);
    robot.finishRun();
    idle();

    //Gyro Corrects:
    gyroCorrect(0, gyroReset, power);
  }
}
