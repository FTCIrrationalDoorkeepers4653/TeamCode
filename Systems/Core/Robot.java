package org.firstinspires.ftc.teamcode.Systems.Core;

import android.graphics.Bitmap;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Systems.Vision.TensorVision;

import java.util.ArrayList;
import java.util.Arrays;

import lib.Positioning;

public class Robot extends LinearOpMode {
  /* DRIVE TRAIN AND MOTOR VARIABLES */

  //Drive Train Variables:
  public static double robotDimensions = 21.0;
  public static double gearRatio = 1.0;
  public static double wheelDiam = 3.9;
  public static double wheelCirc = (Math.PI * wheelDiam);
  public static double TicksPerRev = 560.0;
  public static double POSITION_RATIO = (144.0 / 760.0);

  //Drive Train Augmentation Variables:
  public static double wheelRPM = 100.0;
  public static double wheelRPS = (wheelRPM / 60.0);
  public static double degreesPerTick = (TicksPerRev / 360.0);
  public static double gyroStabilization = 10.0;
  public static double powerControl = 2.0;

  //Motor Powers:
  public static double zeroPower = 0.0;
  public static double slowPower = 0.3;
  public static double movePower = 0.5;
  public static double gyroPower = 0.6;
  public static double firePower = 0.8;
  public static double uncoPower = 1.0;

  /* POSITION VARIABLES */

  //Position Variables:
  private static double roadblockX[] = {0.0, 760.0, 0.0, 760.0};
  private static double roadblockY[] = {0.0, 0.0, 760.0, 760.0};

  /* VISION VARIABLES */
  //Detector Settings:
  private static int detector[] = {255, 120, 0};
  private static int margin[] = {100, 100, 100};
  private static int FSD_MARGIN[] = {50, 50, 50};
  private static boolean flash = true;
  private static int zoom = 20;
  private static int x = 0, y = 0, width = 50, height = 15;
  private static int firstCount = 10, secondCount = 500;
  private static double resizeRatio = 0.2;
  private static String vuforiaKey =
          "AR7KPuz/////AAABmSKvAg58mkBSqvvfxvaYqxMN8S2CvbOIzcpLyLVqb9hLPXQf3hPCERtF9azaj5sBUezFRBqdVA53ZAsNmlWW/" +
                  "ThqkaHtmpKNqXneP6p8VhN4liG3ofA7Cidx234PKNIhalLvby0jdmuxT5Uhh4dJjST6taoZGArAQz7Df8hzPG26Nd92L1A" +
                  "TW3mO4qzNAny2UK5YrzG92bUIxqvpDLkjeq8UNTLHYD4ulI1i+Jl/dPzU2PdeNPEqlsykdshGvcuRWRz8qeMXfpKVZ9TXmLxqvu" +
                  "Te6K291gxuKtfWXJ11rYJHTJlUAvooMpPaAh2/isv6LUy83+3UhIyl1kNxaNeMHK52iqEjpswOiOmVkniWTblp";

  //Vision Positioning Settings:
  private static int frameWidth = 1280, frameHeight = 720;
  private static double distanceRatio = 0.1, offsetRatio = 0.05;
  private static double camOffsetX = 4.0, camOffsetY = 0.0;
  private static int realFrameWidth = (int)(frameWidth * resizeRatio);
  private static int realFrameHeight = (int)(frameHeight * resizeRatio);
  private static int distanceThreshold = 100;

  /* HARDWARE VARIABLES */

  //Wheels:
  public static DcMotor leftDriveMotor;
  public static DcMotor rightDriveMotor;

  //Hardware Objects:
  public static Mechanisms mechanisms = new Mechanisms();
  public static TensorVision vision = new TensorVision();
  public static Positioning positioning = new Positioning();

  //IMU Sensor:
  private static BNO055IMU imu;
  private static Orientation angles;

  /* INITIALIZATION METHODS */

  @Override
  public void runOpMode() {
    /* Hardware Access */
  }

  //Initialization Method:
  public static void init(HardwareMap hardwareMap, boolean camera) {
    /* Hardware */

    //Wheel Maps:
    leftDriveMotor = hardwareMap.dcMotor.get("leftDriveMotor");
    rightDriveMotor = hardwareMap.dcMotor.get("rightDriveMotor");

    /* Motors */

    //Wheel Directions:
    leftDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    rightDriveMotor.setDirection(DcMotor.Direction.REVERSE);

    //Motor Behavior Setup:
    applyAllModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    applyAllModes(DcMotor.RunMode.RUN_USING_ENCODER);
    applyAllZero(DcMotor.ZeroPowerBehavior.BRAKE);

    /* Sensors */

    //IMU Initialization:
    BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
    imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    imu = hardwareMap.get(BNO055IMU.class, "imu");
    imu.initialize(imuParameters);

    //Checks the Case:
    if (camera) {
      //Vision Initialization:
      vision.initVuforia(hardwareMap, vuforiaKey, zoom, flash);
      vision.initDetector("detector", detector);
      vision.initPositioning(realFrameWidth, realFrameHeight, distanceRatio, offsetRatio,
      camOffsetX, camOffsetY);
    }

    /* Positions */

    //Controllers Initialization:
    mechanisms.resetCurrentPosition();
    mechanisms.setRoadblocks(roadblockX, roadblockY);
    applyAllPowers(zeroPower);
  }

  /* VISION METHODS */

  //Gets the Duck in the Frame:
  public static double getDuckPosition() {
    //Position Variable:
    int position = 1;

    //Returns the Position:
    return position;
  }

  /* IMU METHODS */

  //Gets the Rotations of Correction Method:
  public static double getGyroCorrection(double expectedAngle) {
    //Gets IMU Correction:
    double imuReading = getTheta();
    double correction = (expectedAngle - imuReading);

    //Converts and Returns:
    double rotations = calculateTurnRotations(correction);
    return rotations;
  }

  //Gets the Robot Theta Position:
  public static double getTheta() {
    //Returns the Robot Theta:
    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    return angles.firstAngle;
  }

  //Calculates If Value is Within Range:
  public static boolean isWithinRange(double value, double check, double offset) {
    //Main Boolean and Values:
    double max = check + offset;
    double min = check - offset;

    //Checks the Case:
    if (value >= min && value <= max) {
      //Returns the Value:
      return true;
    }

    else {
      //Returns the Value:
      return false;
    }
  }

  /* IMU CALCULATION METHODS */

  //Calculates Turn Rotations in Terms of Degrees:
  public static double calculateTurnRotations(double angleDegrees) {
    double angleInRadians = convertAngle(angleDegrees, false); //Degrees to Radians (for Arc Length)
    double robotRotations = (robotDimensions / wheelCirc); //Width and Height in terms of Rotations
    double d = Math.sqrt((robotRotations * robotRotations) + (robotRotations * robotRotations)); //Diameter Value of Circumscribed Robot
    double r = d / 2.0; //Radius Value of Circumscribed Robot

    /* Ratio is Rotations/Degrees...So to find Rotations it is... x = (ratioRotations*correction)/ratioDegrees */

    //Returns Rotations Needed to Correct Angle (arc length = r * theta):
    double driveRotations = (r * angleInRadians);
    return driveRotations;
  }

  //Calculates Turn Angle from Arc Length:
  public static double calculateTurnAngle(double arc) {
    //Finds Radius of Circle:
    double d = Math.sqrt((robotDimensions * robotDimensions) + (robotDimensions * robotDimensions));
    double r = d / 2.0;

    //Finds and Returns Theta:
    double thetaRadians = (arc / r);
    double thetaDegrees = convertAngle(thetaRadians, true);
    return thetaDegrees;
  }

  //Converts from Radians to Degrees:
  public static double convertAngle(double angle, boolean degrees) {
    //Degrees Double:
    double angleDegrees = ((180.0 / Math.PI) * angle);
    double angleRadians = ((Math.PI / 180.0) * angle);

    //Checks the Case:
    if (degrees) {
      //Returns the Angle:
      return angleDegrees;
    }

    else {
      //Returns the Angle:
      return angleRadians;
    }
  }

  //Calculates Time Required To Complete Operation:
  public static int calculateTime(double rotations, double power) {
    //Calculates the Time and Returns:
    double timeInSeconds = Math.abs((rotations / (wheelRPS * power)));
    int timeInMillis = (int) (timeInSeconds * 1000.0);
    return timeInMillis;
  }

  /* ROBOT MOVEMENT METHODS */

  public void driveRobot(double drive, double turn, boolean control) {
    //Slow Mode:
    if (control) {
      //Sets the New Powers:
      drive /= powerControl;
      turn /= powerControl;
    }

    //Move Robot:
    if (drive != 0.0 && turn == 0.0) {
      //Moves Robot:
      leftDriveMotor.setPower(-drive);
      rightDriveMotor.setPower(-drive);
    }

    else if (drive == 0.0 && turn != 0.0) {
      //Turns Robot:
      leftDriveMotor.setPower(-turn);
      rightDriveMotor.setPower(turn);
    }

    else {
      //Stops Robot:
      leftDriveMotor.setPower(zeroPower);
      rightDriveMotor.setPower(zeroPower);
    }
  }

  //Robot Move with Rotations:
  public void runRobot(double rotations, double power) {
    //Movement Values:
    double localRotations = Math.abs(rotations);
    int parts = getParts(localRotations);

    //Checks the Case:
    if (rotations > 0) {
      //Sets the Target Positions:
      leftDriveMotor.setTargetPosition(leftDriveMotor.getCurrentPosition() + parts);
      rightDriveMotor.setTargetPosition(rightDriveMotor.getCurrentPosition() + parts);
    }

    else if (rotations < 0) {
      //Sets the Target Positions:
      leftDriveMotor.setTargetPosition(leftDriveMotor.getCurrentPosition() - parts);
      rightDriveMotor.setTargetPosition(rightDriveMotor.getCurrentPosition() - parts);
    }

    //Sets the Motor Powers:
    applyAllModes(DcMotor.RunMode.RUN_TO_POSITION);
    applyAllPowers(power);
    sleep(calculateTime(rotations, power));

    //Resets:
    applyAllPowers(zeroPower);
    applyAllModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    applyAllModes(DcMotor.RunMode.RUN_USING_ENCODER);
  }

  //Robot Turn Motion with Rotations:
  public void turnRobot(double rotations, double power) {
    //Movement Values:
    double localRotations = Math.abs(rotations);
    int parts = getParts(localRotations);

    //Checks the Case:
    if (rotations > 0) {
      //Sets the Target Positions:
      leftDriveMotor.setTargetPosition(leftDriveMotor.getCurrentPosition() - parts);
      rightDriveMotor.setTargetPosition(rightDriveMotor.getCurrentPosition() + parts);
    }

    else if (rotations < 0) {
      //Sets the Target Positions:
      leftDriveMotor.setTargetPosition(leftDriveMotor.getCurrentPosition() + parts);
      rightDriveMotor.setTargetPosition(rightDriveMotor.getCurrentPosition() - parts);
    }

    //Sets the Motor Powers and Resets:
    applyAllModes(DcMotor.RunMode.RUN_TO_POSITION);
    applyAllPowers(power);
    sleep(calculateTime(rotations, power));

    //Resets:
    applyAllPowers(zeroPower);
    applyAllModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    applyAllModes(DcMotor.RunMode.RUN_USING_ENCODER);
  }

  /* ROBOT MOTOR UTILITY METHODS */

  //Apply Power to All Motors:
  public static void applyAllPowers(double power) {
    //Applies the Motor Powers:
    leftDriveMotor.setPower(power);
    rightDriveMotor.setPower(power);
  }

  //Apply Mode to All Motors:
  public static void applyAllModes(DcMotor.RunMode mode) {
    //Applies the Motor Modes:
    leftDriveMotor.setMode(mode);
    rightDriveMotor.setMode(mode);
  }

  //Apply Zero Power Behavior to All Motors:
  public static void applyAllZero(DcMotor.ZeroPowerBehavior behavior) {
    //Applies the Motor Behaviors:
    leftDriveMotor.setZeroPowerBehavior(behavior);
    rightDriveMotor.setZeroPowerBehavior(behavior);
  }

  /* ROBOT UTILITY METHODS */

  //Gets Parts Based on Rotations:
  public static int getParts(double rotations) {
    //Calculates and Returns Parts:
    int parts = (int)((rotations * TicksPerRev) * gearRatio);
    return parts;
  }

  //Gets the Rotations Based on Angle:
  public static double getAngleRotations(double angle) {
    //Gets the Rotations and Returns:
    int parts = (int)(angle * degreesPerTick);
    double rotations = (parts / TicksPerRev);
    return rotations;
  }
}