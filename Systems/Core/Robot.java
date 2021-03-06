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

import lib.Positioning;

public class Robot extends LinearOpMode {
  /* DRIVE TRAIN AND MOTOR VARIABLES */

  //Drive Train Variables:
  public static double robotDimensions = 21.0;
  public static double gearRatio = 1.0;
  public static double wheelDiam = 3.9;
  public static double wheelCirc = (Math.PI * wheelDiam);
  public static double TicksPerRev = 1120.0;
  public static double POSITION_RATIO = (144.0 / 760.0);

  //Drive Train Augmentation Variables:
  public static double wheelRPM = 100.0;
  public static double wheelRPS = (wheelRPM / 60.0);
  public static double degreesPerTick = (TicksPerRev / 360.0);
  public static double gyroStabilization = 10.0;
  public static double speedControl = 3.0;

  //Motor Powers:
  public static double zeroPower = 0.0;
  public static double slowPower = 0.3;
  public static double movePower = 0.5;
  public static double gyroPower = 0.6;
  public static double firePower = 0.8;
  public static double uncoPower = 1.0;

  /* POSITION VARIABLES */

  //Position Profiling Variables:
  private static double Kc = 1.0;
  private static double timeOut = 15.0;
  private static double roadblockX[] = {0.0, 760.0, 0.0, 760.0};
  private static double roadblockY[] = {0.0, 0.0, 760.0, 760.0};

  /* VISION VARIABLES */

  //Detector Settings:
  private static int detector[] = {253, 168, 53};
  private static int offset[] = {100, 100, 100};
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
  private static double offsetX = 10.0, offSetY = 14.0;
  private static double distanceOfField = 13.0, camZoom = 1.0;
  private static double fieldDistance = (distanceOfField * camZoom);

  /* HARDWARE VARIABLES */

  //Wheels:
  public static DcMotor leftFrontMotor;
  public static DcMotor leftBackMotor;
  public static DcMotor rightFrontMotor;
  public static DcMotor rightBackMotor;

  //Hardware Objects:
  public static Mechanisms mechanisms = new Mechanisms(Kc, firePower, timeOut);
  public static TensorVision vision = new TensorVision();

  //IMU Sensor:
  private static BNO055IMU imu;
  private static Orientation angles;

  /* INITIALIZATION METHODS */

  @Override
  public void runOpMode() {
    /* Hardware Access */
  }

  //Initialization Method:
  public static void init(HardwareMap hardwareMap, boolean type, boolean camera) {
    /* Hardware */

    //Wheel Maps:
    leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
    leftBackMotor = hardwareMap.dcMotor.get("leftBackMotor");
    rightFrontMotor = hardwareMap.dcMotor.get("rightFrontMotor");
    rightBackMotor = hardwareMap.dcMotor.get("rightBackMotor");

    /* Motors */

    //Wheel Directions:
    leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
    rightBackMotor.setDirection(DcMotor.Direction.FORWARD);

    //Checks the Case:
    if (type) {
      //Motor Behavior Setup:
      applyAllModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      applyAllModes(DcMotor.RunMode.RUN_USING_ENCODER);
      applyAllZero(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    else {
      //Motor Behavior Setup:
      applyAllModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      applyAllModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      applyAllZero(DcMotor.ZeroPowerBehavior.FLOAT);
    }

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
      vision.initDetector("ringDetector", detector);
      vision.initPositioning(frameWidth, frameHeight, fieldDistance, offsetX, offSetY);
    }

    /* Positions */

    //Controllers Initialization:
    mechanisms.resetCurrentPosition();
    mechanisms.setRoadblocks(roadblockX, roadblockY);
    applyAllPowers(zeroPower);
  }

  /* VISION METHODS */

  //Gets the Universal Vision Information:
  public static int getVision() {
    //Analyzes and Returns the Boolean Count:
    Bitmap image = vision.getImage(resizeRatio);
    int rgb[][] = vision.getBitmapRGB(image, x, y, width, height);
    int booleanCount = vision.detectPixelCount(rgb, offset, 0);
    return booleanCount;
  }

  //Gets the Position Pixel Count:
  public static int getPixelsPosition() {
    //Vision Positions:
    int position = 1;
    int booleanCount = getVision();

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

  //Robot Move with Rotations:
  public void runRobot(double rotations, double power) {
    //Movement Values:
    double localRotations = Math.abs(rotations);
    int parts = getParts(localRotations);

    //Checks the Case:
    if (rotations > 0) {
      //Sets the Target Positions:
      leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() + parts);
      leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() + parts);
      rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() + parts);
      rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() + parts);
    }

    else if (rotations < 0) {
      //Sets the Target Positions:
      leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() - parts);
      leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() - parts);
      rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() - parts);
      rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() - parts);
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
      leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() - parts);
      leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() - parts);
      rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() + parts);
      rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() + parts);
    }

    else if (rotations < 0) {
      //Sets the Target Positions:
      leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() + parts);
      leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() + parts);
      rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() - parts);
      rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() - parts);
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

  //Robot Strafe Motion:
  public void shiftRobot(double rotations, double power) {
    //Movement Values:
    double localRotations = Math.abs(rotations);
    int parts = getParts(localRotations);

    //Checks the Case:
    if (rotations > 0) {
      //Sets the Target Positions:
      leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() - parts);
      leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() + parts);
      rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() + parts);
      rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() - parts);
    }

    else if (rotations < 0) {
      //Sets the Target Positions:
      leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() + parts);
      leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() - parts);
      rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() - parts);
      rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() + parts);
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

  /* ROBOT ENCODER UTILITY METHODS */

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

  //Gets the Speed Based on Slow Parameters:
  public static double getSpeedControl(double speed, boolean control) {
    //Checks the Case:
    if (control) {
      //Returns the Speed:
      return (speed / speedControl);
    }

    else {
      //Returns the Speed:
      return speed;
    }
  }
}