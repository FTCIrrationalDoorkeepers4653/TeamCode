package org.firstinspires.ftc.teamcode.Systems.Core;

import android.graphics.Bitmap;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Systems.Vision.TensorVision;

public class Robot {
  /* HARDWARE VARIABLES */

  //Wheels:
  public static DcMotor leftFrontMotor;
  public static DcMotor leftBackMotor;
  public static DcMotor rightFrontMotor;
  public static DcMotor rightBackMotor;

  //IMU Sensor:
  private static BNO055IMU imu;
  private static Orientation angles;

  //Hardware Objects:
  public static HardwareMap hardwareMap;
  public static Controller controller = new Controller();
  public static Mechanisms mechanisms = new Mechanisms();
  public static TensorVision vision = new TensorVision();

  /* DRIVE TRAIN AND MOTOR VARIABLES */

  //Drive Train Variables:
  public static double robotDimensions = 17.8;
  public static double gearRatio = 1.0;
  public static double wheelDiam = 3.9;
  public static double wheelCirc = (Math.PI * wheelDiam);
  public static double TicksPerRev = 1120;
  public static double POSITION_RATIO = (144.0 / 760.0);
  public static double wheelRPM = 100.0;
  public static double wheelRPS = (wheelRPM / 60.0);
  public static double gyroStabilization = 0.3;
  public static double degreesPerTick = (360.0 / TicksPerRev);
  public static double timeAdjustment = 0.5;
  public static double speedControl = 3.0;

  //Motor Powers:
  public static double zeroPower = 0.0;
  public static double slowPower = 0.4;
  public static double mainPower = 0.5;
  public static double fastPower = 0.6;
  public static double uncoPower = 0.8;
  public static double maxiPower = 1.0;
  public static double contPower = 2.0;

  /* VISION VARIABLES */

  //Detector Settings:
  private static int detector[] = {253, 168, 53};
  private static int offset[] = {100, 100, 100};
  private static boolean flash = true;
  private static int zoom = 20;
  private static int x = 0, y = 0, width = 51, height = 26;
  private static int firstCount = 20, secondCount = 400;
  private static double resizeRatio = 0.2;
  private static String vuforiaKey =
  "AR7KPuz/////AAABmSKvAg58mkBSqvvfxvaYqxMN8S2CvbOIzcpLyLVqb9hLPXQf3hPCERtF9azaj5sBUezFRBqdVA53ZAsNmlWW/" +
  "ThqkaHtmpKNqXneP6p8VhN4liG3ofA7Cidx234PKNIhalLvby0jdmuxT5Uhh4dJjST6taoZGArAQz7Df8hzPG26Nd92L1A" +
  "TW3mO4qzNAny2UK5YrzG92bUIxqvpDLkjeq8UNTLHYD4ulI1i+Jl/dPzU2PdeNPEqlsykdshGvcuRWRz8qeMXfpKVZ9TXmLxqvu" +
  "Te6K291gxuKtfWXJ11rYJHTJlUAvooMpPaAh2/isv6LUy83+3UhIyl1kNxaNeMHK52iqEjpswOiOmVkniWTblp";

  /* HARDWARE INITIALIZATION METHODS */

  //Initialization Method:
  public static void init(HardwareMap hwMap, boolean type, boolean camera) {
    /* Hardware */

    //Save reference to Hardware map:
    hardwareMap = hwMap;

    //Wheel Maps:
    leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
    leftBackMotor = hardwareMap.dcMotor.get("leftBackMotor");
    rightFrontMotor = hardwareMap.dcMotor.get("rightFrontMotor");
    rightBackMotor = hardwareMap.dcMotor.get("rightBackMotor");

    /* Sensors */

    //Checks the Case:
    if (camera) {
      //Vision Initialization:
      vision.initVuforia(hardwareMap, vuforiaKey, zoom, flash);
      vision.initDetector("", detector);
    }

    //IMU Initialization:
    BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
    imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    imu = hardwareMap.get(BNO055IMU.class, "imu");
    imu.initialize(imuParameters);

    /* Motors */

    //Checks the Case:
    if (type) {
      //Wheel Setup:
      applyAllPowers(zeroPower);
      applyAllModes(DcMotor.RunMode.RUN_USING_ENCODER);
      applyAllZero(DcMotor.ZeroPowerBehavior.FLOAT);

      //Wheel Directions:
      leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
      leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
      rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
      rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    else {
      //Wheel Setup:
      applyAllPowers(zeroPower);
      applyAllModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      applyAllZero(DcMotor.ZeroPowerBehavior.FLOAT);

      //Wheel Directions:
      leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
      leftBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
      rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
      rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
    }
  }

  /* VISION METHODS */

  //Gets the Position Pixel Count:
  public static int getPixelsPosition() {
    //Position Variable and Image (w/ Default):
    int position = 1;
    Bitmap image = vision.getImage(resizeRatio);

    //Gets the Boolean Count:
    int rgb[][] = vision.getBitmapRGB(image, x, y, width, height);
    int booleanCount = vision.detectPixelCount(rgb, offset, 0);

    //Checks the Case:
    if (booleanCount <= firstCount) {
      //Sets the Position:
      position = 1;
    }

    else if (booleanCount > firstCount && booleanCount <= secondCount) {
      //Sets the Position:
      position = 2;
    }

    else {
      //Sets the Position:
      position = 3;
    }

    //Returns the Position:
    return position;
  }

  /* IMU METHODS */

  //Calculates If Value is Within Range:
  public static boolean isWithinRange(double value, double initial, double offset) {
    //Main Boolean and Values:
    boolean within = false;
    double max = initial + (initial * offset);
    double min = initial - (initial * offset);

    //Checks the Case:
    if (value >= min && value <= max) {
      within = true;
    }

    //Returns the Boolean:
    return within;
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

  //Calculates Turn Rotations in Terms of Degrees:
  public static double calculateTurnRotations(double angleDegrees) {
    double angleInRadians = convertAngle(angleDegrees, false); //Degrees to Radians (for Arc Length)
    double robotRotations = (robotDimensions / wheelCirc); //Width and Height in terms of Rotations
    double d = Math.sqrt((robotRotations * robotRotations) + (robotRotations * robotRotations)); //Diameter Value of Circumscribed Robot
    double r = d / 2.0; //Radius Value of Circumscribed Robot

    /* Ratio is Rotations/Degrees...So to find Rotations it is... x = (ratioRotations*correction)/ratioDegrees */

    //Rotations Needed to Correct Angle (arc length = r * theta):
    double driveRotations = (r * angleInRadians);

    //Returns the Value:
    return driveRotations;
  }

  //Gets the Rotations of Correction Method:
  public static double[] getGyroCorrection(double expectedAngle, double resetValue) {
    //Main Array and Variables:
    double directionAndRotations[] = new double[2];
    double direction = 0.0, rotations = 0.0;

    //Gets IMU Reading:
    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    double imuReading = angles.firstAngle;
    double imuResetReading = imuReading + resetValue;

    //Checks the Case:
    if (isWithinRange(imuReading, expectedAngle, gyroStabilization)) {
      //Checks the Case:
      if (imuResetReading > expectedAngle) {
        //Sets the Heading:
        direction = -1;
      }

      else if (imuResetReading < expectedAngle) {
        //Sets the Heading:
        direction = 1;
      }

      //Calculates Correction:
      double correction = Math.abs(Math.abs(expectedAngle) - Math.abs(imuResetReading));
      rotations = calculateTurnRotations(correction);
    }

    //Formats the Return Array:
    directionAndRotations[0] = direction;
    directionAndRotations[1] = rotations;

    //Returns the Direction (-1, 0, or 1) and the Rotations:
    return directionAndRotations;
  }

  //Resets Gyro Sensor Value:
  public static double resetGyroValue() {
    //Gets Current Value and Returns Reset:
    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    double currentIMUReading = (double)(angles.firstAngle);
    double resetValue = 0 - currentIMUReading;
    return resetValue;
  }

  /* ROBOT MOVEMENT METHODS */

  //Resets Encoders After Every Movement:
  public static void finishRun() {
    //Sets the Motors:
    applyAllPowers(zeroPower);
    applyAllModes(DcMotor.RunMode.RUN_USING_ENCODER);
  }

  //Robot Move with Rotations:
  public static void runRobot(String type, double rotations, double power) {
    //Checks the Case:
    if (type.equalsIgnoreCase("forward")) {
      //Sets the Target Positions:
      leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() + getParts(rotations));
      leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() + getParts(rotations));
      rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() + getParts(rotations));
      rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() + getParts(rotations));
    }

    else if (type.equalsIgnoreCase("backward")) {
      //Sets the Target Positions:
      leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() - getParts(rotations));
      leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() - getParts(rotations));
      rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() - getParts(rotations));
      rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() - getParts(rotations));
    }

    //Sets the Motors:
    applyAllModes(DcMotor.RunMode.RUN_TO_POSITION);
    controller.applyControlPower(power);
  }

  //Robot Turn Motion with Rotations:
  public static void turnRobot(String type, double rotations, double power) {
    //Checks the Case:
    if (type.equalsIgnoreCase("left")) {
      //Sets the Target Positions:
      leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() - getParts(rotations));
      leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() - getParts(rotations));
      rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() + getParts(rotations));
      rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() + getParts(rotations));
    }

    else if (type.equalsIgnoreCase("right")) {
      //Sets the Target Positions:
      leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() + getParts(rotations));
      leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() + getParts(rotations));
      rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() - getParts(rotations));
      rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() - getParts(rotations));
    }

    //Sets the Motors:
    applyAllModes(DcMotor.RunMode.RUN_TO_POSITION);
    controller.applyControlPower(power);
  }

  //Robot Strafe Motion:
  public static void shiftRobot(String type, double rotations, double power) {
    //Checks the Case:
    if (type.equalsIgnoreCase("left")) {
      //Sets the Target Positions:
      leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() - getParts(rotations));
      leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() + getParts(rotations));
      rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() + getParts(rotations));
      rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() - getParts(rotations));
    }

    else if (type.equalsIgnoreCase("right")) {
      //Sets the Target Positions:
      leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() + getParts(rotations));
      leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() - getParts(rotations));
      rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() - getParts(rotations));
      rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() + getParts(rotations));
    }

    //Sets the Motors:
    applyAllModes(DcMotor.RunMode.RUN_TO_POSITION);
    controller.applyControlPower(power);
  }

  /* ROBOT UTILITY METHODS */

  //Apply Power to All Motors:
  public static void applyAllPowers(double power) {
    //Applies the Motor Powers:
    leftFrontMotor.setPower(power);
    leftBackMotor.setPower(power);
    rightFrontMotor.setPower(power);
    rightBackMotor.setPower(power);
  }

  //Apply Mode to All Motors:
  public static void applyAllModes(DcMotor.RunMode mode) {
    //Applies the Motor Modes:
    leftFrontMotor.setMode(mode);
    leftBackMotor.setMode(mode);
    rightFrontMotor.setMode(mode);
    rightBackMotor.setMode(mode);
  }

  //Apply Zero Power Behavior to All Motors:
  public static void applyAllZero(DcMotor.ZeroPowerBehavior behavior) {
    //Applies the Motor Behaviors:
    leftFrontMotor.setZeroPowerBehavior(behavior);
    leftBackMotor.setZeroPowerBehavior(behavior);
    rightFrontMotor.setZeroPowerBehavior(behavior);
    rightBackMotor.setZeroPowerBehavior(behavior);
  }

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