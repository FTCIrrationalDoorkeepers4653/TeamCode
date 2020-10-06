package org.firstinspires.ftc.teamcode.Systems.Movement;

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
import org.firstinspires.ftc.teamcode.Systems.Vision.Camera;
import org.firstinspires.ftc.teamcode.Systems.Vision.Yellow;

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
  public static double mainPower = 0.6;
  public static double fastPower = 0.8;

  //Objects:
  public static HardwareMap hardwareMap;
  public static Camera camera = new Camera();
  public static Yellow yellow = new Yellow();
  public static Positions positions = new Positions();

  //Detector Settings:
  public static int detector[] = {255, 215, 0}, offset[] = {100, 100, 100};
  public static String detectorName = "Ring Detector";
  public static boolean turnOnFlash = false;
  public static int zoomInit = 1;
  public static int x = 150, y = 40, width = 106, height = 64;
  public static int firstCount = 20, secondCount = 140, pixelCount = 280;
  public static double resizeRatio = 0.2;

  /* INITIALIZATION AND CONTROL */

  //Initialization Method:
  public static void init(HardwareMap hwMap, boolean type) {
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

    //Vision Initialization:
    camera.initVuforia(hardwareMap, zoomInit, turnOnFlash);
    camera.setDetector(detectorName, detector);

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
    Bitmap image = camera.getImage(resizeRatio);

    //Gets the Boolean Count:
    int rgb[][] = camera.getBitmapRGB(image, x, y, width, height);
    int booleanCount = camera.detectPixelCount(rgb, offset, pixelCount);

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

  /* IMU Gyro Correction Method is Used in Conjunction with Turn Method, Reset Gyro Before Use:
     (PARAMS-expectedAngle should always be with direction [+ -> left and - -> right]) */
  public static double[] getGyroCorrection(double expectedAngle, double resetValue) {
    //Main Array and Variables:
    double directionAndRotations[] = new double[2];
    double direction = 0;

    //Gets IMU Reading:
    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    double imuReading = angles.firstAngle;

    //Gets Correct IMU Reset Reading:
    double imuResetReading = imuReading + resetValue;

    //Sets the Direction:
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
    double currentIMUReading = (double) angles.firstAngle;
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