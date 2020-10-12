package org.firstinspires.ftc.teamcode.Systems.Core;

import android.graphics.Bitmap;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Systems.Vision.TensorVision;
import org.firstinspires.ftc.teamcode.Systems.Vision.DogeVision;
import org.firstinspires.ftc.teamcode.Systems.Vision.Lib.VisionPosition;

public class Robot {
  /* SYSTEM VARIABLES */

  //Wheels:
  public static DcMotor leftFrontMotor;
  public static DcMotor leftBackMotor;
  public static DcMotor rightFrontMotor;
  public static DcMotor rightBackMotor;

  //Mechanisms:
  public static DcMotor baseArmMotor;

  //Servos:
  public static Servo holdServo;
  public static Servo clawServo;

  //IMU Sensor:
  public static BNO055IMU imu;
  public static Orientation angles;

  //Drive Train Variables:
  public static double robotDimensions = 17.8;
  public static double gearRatio = 1.0;
  public static double wheelDiam = 3.9;
  public static double wheelCirc = (Math.PI * wheelDiam);
  public static double TicksperRev = 1120;
  public static double POSITION_RATIO = (144.0 / 760.0);
  public static double wheelRPM = 100.0;
  public static double wheelRPS = (wheelRPM / 60.0);

  //Mechanism Variables:
  public static double clawServoStartPosition = 0.0;
  public static double holdServoStartPosition = 0.0;
  public static double clawServoEndPosition = 1.0;
  public static double holdServoEndPosition = 0.6;

  //Powers:
  public static double zeroPower = 0.0;
  public static double slowPower = 0.4;
  public static double mainPower = 0.5;
  public static double fastPower = 0.6;
  public static double speedControl = 3.0;

  //Objects:
  public static HardwareMap hardwareMap;
  public static VisionPosition positioning = new VisionPosition();
  public static DogeVision doge = new DogeVision();
  public static TensorVision vision = new TensorVision();
  public static Positions positions = new Positions();

  //Detector Settings:
  public static int detector[] = {255, 215, 0}, offset[] = {100, 100, 100};
  public static String detectorName = "Ring Detector";
  public static boolean flash = true;
  public static int zoom = 20;
  public static int x = 0, y = 0, width = 51, height = 26;
  public static int firstCount = 20, secondCount = 80, pixelCount = 280;
  public static double resizeRatio = 0.2;
  private static String VUFORIA_KEY =
  "AR7KPuz/////AAABmSKvAg58mkBSqvvfxvaYqxMN8S2CvbOIzcpLyLVqb9hLPXQf3hPCERtF9azaj5sBUezFRBqdVA53ZAsNmlWW/" +
  "ThqkaHtmpKNqXneP6p8VhN4liG3ofA7Cidx234PKNIhalLvby0jdmuxT5Uhh4dJjST6taoZGArAQz7Df8hzPG26Nd92L1A" +
  "TW3mO4qzNAny2UK5YrzG92bUIxqvpDLkjeq8UNTLHYD4ulI1i+Jl/dPzU2PdeNPEqlsykdshGvcuRWRz8qeMXfpKVZ9TXmLxqvu" +
  "Te6K291gxuKtfWXJ11rYJHTJlUAvooMpPaAh2/isv6LUy83+3UhIyl1kNxaNeMHK52iqEjpswOiOmVkniWTblp";

  //Vision Position Settings:
  public static int referencePixel[] = {0, width / 2};
  public static double referenceDistance = 144.0, camOffsetX = 4.0, camOffsetY = 12.0;

  /* INITIALIZATION AND CONTROL */

  //Initialization Method:
  public static void init(HardwareMap hwMap, boolean type, boolean camera) {
    //Save reference to Hardware map:
    hardwareMap = hwMap;

    //Wheel Maps:
    leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
    leftBackMotor = hardwareMap.dcMotor.get("leftBackMotor");
    rightFrontMotor = hardwareMap.dcMotor.get("rightFrontMotor");
    rightBackMotor = hardwareMap.dcMotor.get("rightBackMotor");

    //Mechanism Maps:
    baseArmMotor = hardwareMap.dcMotor.get("baseArmMotor");
    clawServo = hardwareMap.servo.get("clawServo");
    holdServo = hardwareMap.servo.get("holdServo");

    //Checks the Case:
    if (camera) {
      //Vision Initialization:
      vision.initVuforia(hardwareMap, VUFORIA_KEY, zoom, flash);
      vision.initDetector(detectorName, detector);
      positioning.initVisionPosition(referencePixel, referenceDistance, camOffsetX, camOffsetY);
    }

    //IMU Initialization:
    BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
    imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    imu = hardwareMap.get(BNO055IMU.class, "imu");
    imu.initialize(imuParameters);

    //Checks the Case:
    if (type) {
      //Motor Encoders:
      leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      baseArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    else {
      //Motor Encoders:
      leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      baseArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Motor Directions:
    leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
    rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
    baseArmMotor.setDirection(DcMotor.Direction.FORWARD);
  }

  /* VISION METHODS */

  //Gets the Position Pixel Count:
  public static int getPixelsPosition() {
    //Position Variable and Image (w/ Default):
    int position = 1;
    Bitmap image = vision.getImage(resizeRatio);

    //Gets the Boolean Count:
    int rgb[][] = vision.getBitmapRGB(image, x, y, width, height);
    int booleanCount = vision.detectPixelCount(rgb, offset, pixelCount);

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

  /* CALCULATION METHODS */

  //Calculates Time Required To Complete Operation:
  public static int calculateTime(double rotations, double power) {
    //Calculates the Time Using Ratio:
    double timeInSeconds = Math.abs((rotations / (wheelRPS * power)));
    int timeInMillis = (int) (timeInSeconds * 1000.0);

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
    double theta = convertAngle(Math.abs((Math.atan(x / y))), true);

    //Sets the Values (x, y, h, theta):
    triangle[0] = x;
    triangle[1] = y;
    triangle[2] = h;
    triangle[3] = theta;

    //Returns the Array:
    return triangle;
  }

  //Get Converted Rotations:
  public static double getConvertedRotations(double pixelDistance) {
    //Converts and Returns Data:
    double coordinatesToInches = (pixelDistance * POSITION_RATIO);
    double inchesToRotations = (coordinatesToInches / wheelCirc);
    return inchesToRotations;
  }

  //Step Function Control Method:
  public static double getStepFunction(double value) {
    //Main Step Function Output:
    double output = 0.0;

    //Checks the Case:
    if (value > 0.0) {
      //Sets the Output:
      output = 1.0;
    }

    else if (value < 0.0) {
      //Sets the Output:
      output = -1.0;
    }

    else {
      //Sets the Output:
      output = 0.0;
    }

    //Returns the Output:
    return output;
  }

  /* IMU METHODS */

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
    double direction = 0;

    //Gets IMU Reading:
    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    double imuReading = angles.firstAngle;
    double imuResetReading = imuReading + resetValue;

    //Checks the Case:
    if (imuResetReading > expectedAngle) {
      //Sets the Heading:
      direction = -1;
    }

    else if (imuResetReading < expectedAngle) {
      //Sets the Heading:
      direction = 1;
    }

    //Calculates Correction Degrees:
    directionAndRotations[0] = direction;
    double correction = Math.abs(Math.abs(expectedAngle) - Math.abs(imuResetReading));

    //Calculates Correction Rotations:
    double driveRotations = calculateTurnRotations(correction);
    directionAndRotations[1] = driveRotations;

    //Returns the Direction (-1, 0, or 1) and the Rotations:
    return directionAndRotations;
  }

  //Resets Gyro Sensor Value:
  public static double resetGyroValue() {
    //Gets Current Value and Resets:
    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    double currentIMUReading = (double)(angles.firstAngle);
    double resetValue = 0 - currentIMUReading;

    //Returns the Zero Value:
    return resetValue;
  }

  /* ROBOT MOVEMENT METHODS */

  //Resets Encoders After Every Movement:
  public static void finishRun() {
    //Resets Power:
    leftFrontMotor.setPower(zeroPower);
    leftBackMotor.setPower(zeroPower);
    rightFrontMotor.setPower(zeroPower);
    rightBackMotor.setPower(zeroPower);
    baseArmMotor.setPower(zeroPower);

    //Runs Using Encoders:
    leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    baseArmMotor.setMode((DcMotor.RunMode.RUN_USING_ENCODER));
  }

  //Robot Move with Rotations:
  public static void runRobot(String type, double distanceInRotations, double power) {
    //Calculates Encoder Values:
    int parts = (int) ((distanceInRotations * TicksperRev) * gearRatio);

    //Checks the Case:
    if (type.equalsIgnoreCase("forward")) {
      //Sets the Target Positions:
      leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() + parts);
      leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() + parts);
      rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() + parts);
      rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() + parts);
    }

    else if (type.equalsIgnoreCase("backward")) {
      //Sets the Target Positions:
      leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() - parts);
      leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() - parts);
      rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() - parts);
      rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() - parts);
    }

    //Changes Motor Mode:
    leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    //Runs Motors to Position:
    leftFrontMotor.setPower(power);
    leftBackMotor.setPower(power);
    rightFrontMotor.setPower(power);
    rightBackMotor.setPower(power);
  }

  //Robot Turn Motion with Rotations:
  public static void turnRobot(String type, double distanceInRotations, double power) {
    //Calculates Encoder Values:
    int parts = (int) ((distanceInRotations * TicksperRev) * gearRatio);

    //Checks the Case:
    if (type.equalsIgnoreCase("left")) {
      //Sets the Target Positions:
      leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() - parts);
      leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() - parts);
      rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() + parts);
      rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() + parts);
    }

    else if (type.equalsIgnoreCase("right")) {
      //Sets the Target Positions:
      leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() + parts);
      leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() + parts);
      rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() - parts);
      rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() - parts);
    }

    //Sets the Motor Mode:
    leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    //Runs Motors to Position:
    leftFrontMotor.setPower(power);
    leftBackMotor.setPower(power);
    rightFrontMotor.setPower(power);
    rightBackMotor.setPower(power);
  }

  //Robot Strafe Motion:
  public static void shiftRobot(String type, double distanceInRotations, double power) {
    //Calculates Encoder Values:
    int parts = (int) ((distanceInRotations * TicksperRev) * gearRatio);

    //Checks the Case:
    if (type.equalsIgnoreCase("left")) {
      //Sets the Target Positions:
      leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() - parts);
      leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() + parts);
      rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() + parts);
      rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() - parts);
    }

    else if (type.equalsIgnoreCase("right")) {
      //Sets the Target Positions:
      leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() + parts);
      leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() - parts);
      rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() - parts);
      rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() + parts);
    }

    //Sets the Motor Mode:
    leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    //Runs Motor to Position:
    leftFrontMotor.setPower(power);
    leftBackMotor.setPower(power);
    rightFrontMotor.setPower(power);
    rightBackMotor.setPower(power);
  }

  /* MECHANISM METHODS */

  //Move Arm:
  public static void moveArm(String type, double distanceInRotations, double power) {
    //Calculates Encoder Values:
    int parts = (int) ((distanceInRotations * TicksperRev) * gearRatio);

    //Checks the Case:
    if (type.equalsIgnoreCase("up")) {
      //Sets the Target Positions:
      baseArmMotor.setTargetPosition(baseArmMotor.getCurrentPosition() - parts);
    }

    else if (type.equalsIgnoreCase("down")) {
      //Sets the Target Positions:
      baseArmMotor.setTargetPosition(baseArmMotor.getCurrentPosition() + parts);
    }

    //Runs Motor to Position:
    baseArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    baseArmMotor.setPower(power);
  }

  //Operate Claw:
  public static void operateClaw(String type) {
    //Checks the Case:
    if (type.equalsIgnoreCase("open")) {
      //Sets the Positions:
      clawServo.setPosition(clawServoStartPosition);
      holdServo.setPosition(holdServoStartPosition);
    }

    else if (type.equalsIgnoreCase("close")) {
      //Sets the Positions:
      clawServo.setPosition(clawServoEndPosition);
      holdServo.setPosition(holdServoEndPosition);
    }
  }
}