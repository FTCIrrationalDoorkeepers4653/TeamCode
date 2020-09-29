package org.firstinspires.ftc.teamcode.Systems.Movement;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class PositionMovement extends LinearOpMode {
  //Objects:
  IDKRobot autoRobot = new IDKRobot();

  /* Global Variables: */

  //Final Set Variables:
  private static final double POSITION_RATIO = (144 / 760); //Inches to Pixel Position
  private static final double WHEEL_DIAM = 3.9;
  private static final double WHEEL_CIRC = (Math.PI * WHEEL_DIAM);
  private static final double secondsRatio = (1.5 / 0.8); //(Rotations * time)/(power)

  //Movement Variables (w/ Defaults):
  private static double movementOneRotations = 0, movementTwoRotations = 0;
  private static double globalTheta = 0;

  /* METHODS */

  /* NO POSITION MOVEMENT WILL BE MORE THAN TWO ROBOT MOVEMENTS LONG!!! */
  /* ROBOT IS ASSUMED TO BE FACING NORTH ^ IN MOVEMENTS!!! */

  //Finds the Best Path to Coordinate (Uses AutonomousMapping Application Dimensions 760x760):
  public void findPath(String path[], double startCoordinates[], double endCoordinates[], boolean turnFirst) {
    /* Uses Mechanum Drive Efficiency */

    //Defines the Dimensions of Triangle:
    double triangle[] = getTriangle(startCoordinates, endCoordinates);

    //Defines the Values based on the Path (Movement 1):
    if (path[0].equalsIgnoreCase("left") || path[0].equalsIgnoreCase("right")) {
      if (turnFirst) {
        //Calculates the Turn Angle:
        double thetaInRotations = autoRobot.calculateTurnRotations(triangle[3]);
        movementOneRotations = thetaInRotations;

        //Setting Global Theta Value:
        if (path[0].equalsIgnoreCase("left")) {
          globalTheta = triangle[3];
        } else if (path[0].equalsIgnoreCase("right")) {
          globalTheta = -triangle[3];
        }
      } else { //Shifting First is Discouraged
        //Calculates the Shift Movement:
        double coordinatesToInches = (triangle[0] * POSITION_RATIO);
        double inchesToRotations = (coordinatesToInches / WHEEL_CIRC);
        movementOneRotations = inchesToRotations;
      }
    } else if (path[0].equalsIgnoreCase("forward") || path[0].equalsIgnoreCase("backward")) {
      //Calculates Forwards/Backwards Rotations:
      double coordinatesToInches = (triangle[1] * POSITION_RATIO);
      double inchesToRotations = (coordinatesToInches / WHEEL_CIRC);
      movementOneRotations = inchesToRotations;
    }

    //Defines the Values based on the Path (Movement 2):
    if (path[1].equalsIgnoreCase("left") || path[1].equalsIgnoreCase("right")) {
      //Calculates the Shift Movement:
      double coordinatesToInches = (triangle[0] * POSITION_RATIO);
      double inchesToRotations = (coordinatesToInches / WHEEL_CIRC);
      movementTwoRotations = inchesToRotations;
    } else if (path[1].equalsIgnoreCase("forward") || path[1].equalsIgnoreCase("backward")) {
      //Calculates Forwards/Backwards Rotations:
      double coordinatesToInches = (triangle[1] * POSITION_RATIO);
      double inchesToRotations = (coordinatesToInches / WHEEL_CIRC);
      movementTwoRotations = inchesToRotations;
    }
  }

  //Move to Position Method (Uses AutonomousMapping Application Dimensions 760x760):
  public void moveToPosition(String path[], double power, boolean turnFirst) {
    /* Gyro Correction Outisde of Positioning */

    //Movement One Time:
    int realTimeMillisOne = calculateTime(movementOneRotations, power);

    //Movement Two Times:
    int realTimeMillisTwo = calculateTime(movementTwoRotations, power);

    //Movement One Code:
    if (path[0].equalsIgnoreCase("left") || path[0].equalsIgnoreCase("right")) {
      if (turnFirst) {
        //Makes the Gyro Turn:
        turnGyro(path[0], globalTheta, power);
      } else { //Shifting First is Discouraged
        //Makes Shift Gyro:
        shiftGyro(path[0], movementOneRotations, power);
      }
    } else if (path[0].equalsIgnoreCase("forward") || path[0].equalsIgnoreCase("backward")) {
      //Turns the Robot Towards Position:
      autoRobot.runRobot(path[0], movementOneRotations, power);
      idle();
      sleep(realTimeMillisOne);
      autoRobot.finishRun();
      idle();
    }

    //Movement Two Code:
    if (path[1].equalsIgnoreCase("left") || path[1].equalsIgnoreCase("right")) {
      //Makes Shift Gyro:
      shiftGyro(path[1], movementTwoRotations, power);
    } else if (path[1].equalsIgnoreCase("forward") || path[1].equalsIgnoreCase("backward")) {
      //Runs the Robot Towards Position:
      autoRobot.runRobot(path[1], movementTwoRotations, power);
      idle();
      sleep(realTimeMillisTwo);
      autoRobot.finishRun();
      idle();
    }
  }

  //Movement Position Diagonal (left and up, left and back, right and up, right and back):
  public void moveToDiagonalPosition(String direction, double startCoordinates[], double endCoordinates[], double power) {
    //Uses the Triangle Method:
    double triangle[] = getTriangle(startCoordinates, endCoordinates);

    //Rotations:
    double hInInches = (triangle[2] * POSITION_RATIO);
    double hInRotations = (hInInches / WHEEL_CIRC);

    //Defines Time Needed to Move to Position:
    int realTimeMillis = calculateTime(hInRotations, power);

    //Diagonal Movement:
    autoRobot.diagRobot(direction, hInRotations, power);
    idle();
    sleep(realTimeMillis);
    autoRobot.finishRun();
    idle();
  }

  //Calculates Time Required To Complete Operation:
  public static int calculateTime(double rotations, double power) {
    //Calculates the Time Using Ratio:
    double timeInSeconds = ((secondsRatio * power) / (rotations));
    int timeInMillis = (int) (timeInSeconds * 1000);

    //Returns the Value:
    return timeInMillis;
  }

  //Gets Correct Gyro Correction Angle:
  public double[] getTriangle(double startCoordinates[], double endCoordinates[]) {
    //Main Array:
    double triangle[] = new double[4];

    //Defines the Dimensions of Triangle:
    double x = endCoordinates[0] - startCoordinates[0];
    double y = endCoordinates[1] - startCoordinates[1];
    double h = Math.sqrt(((x * x) + (y * y)));

    //Finds the Angle Necessary:
    double theta = Math.atan(y / x);

    //Sets the Values (x, y, h, theta):
    triangle[0] = x;
    triangle[1] = y;
    triangle[2] = h;
    triangle[3] = theta;

    //Returns the Array:
    return triangle;
  }

  //Gyro Correction Method (Reset Gyro Before):
  public void gyroCorrect(double expectedAngle, double resetValue, double power) {
    //Gyro Correction Calculation:
    double gyroValues[] = autoRobot.getGyroCorrection(expectedAngle, resetValue);
    String turnValue = ""; //Default Value

    //Sets the turn Value:
    if (gyroValues[0] == -1) {
      turnValue = "right";
    } else if (gyroValues[0] == 1) {
      turnValue = "left";
    }

    //Calculates Time Required:
    int millis = calculateTime(gyroValues[1], power);

    //Makes Correction:
    autoRobot.turnRobot(turnValue, gyroValues[1], power);
    idle();
    sleep(millis);
    autoRobot.finishRun();
    idle();
  }

  //Makes turns with Gyro Corrections:
  public void turnGyro(String direction, double angle, double power) {
    //Resets Gyro:
    double gyroResetDeliver = autoRobot.resetGyroValue();

    //Caluclates Turn:
    double turnValueDeliver = autoRobot.calculateTurnRotations(angle);

    //Calculates Time:
    int timeRequiredDeliver = calculateTime(turnValueDeliver, power);

    //Turns Robot:
    autoRobot.turnRobot(direction, turnValueDeliver, power);
    idle();
    sleep(timeRequiredDeliver);
    autoRobot.finishRun();
    idle();

    //Gyro Corrects Robot:
    gyroCorrect(angle, gyroResetDeliver, power);
  }

  //Shifts with Gyro Corrects:
  public void shiftGyro(String direction, double rotations, double power) {
    //Resets Gyro Value:
    double gyroReset = autoRobot.resetGyroValue();

    //Calculates Time:
    int millis = calculateTime(rotations, power);

    //Shifts the Robot Towards Position:
    autoRobot.shiftRobot(direction, movementOneRotations, power);
    idle();
    sleep(millis);
    autoRobot.finishRun();
    idle();

    //Gyro Corrects:
    gyroCorrect(0, gyroReset, power);
  }

  //Placeholder RunOpMode Method:
  @Override
  public void runOpMode() {
    /* Just to Access LinearOpMode Methods */
  }
}
