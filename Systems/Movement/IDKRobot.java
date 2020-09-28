package org.firstinspires.ftc.teamcode.Systems.Movement;

import android.content.Context;
import android.graphics.Bitmap;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Systems.Detectors.VuforiaImageInit;
import org.firstinspires.ftc.teamcode.Systems.Detectors.Yellow;

public class IDKRobot {
  /* SYSTEM VARIABLES */

  //Wheels:
  public DcMotor leftFrontMotor;
  public DcMotor leftBackMotor;
  public DcMotor rightFrontMotor;
  public DcMotor rightBackMotor;

  //Mechanisms:
  public DcMotor baseArmMotor;

  //Servos:
  public Servo holdServo;
  public Servo clawServo;

  //IMU Sensor:
  BNO055IMU imu;
  Orientation angles;

  //Drive Train Variables:
  private static final double robotDimensions = 17.8;
  private static final double gearRatio = 1.0;
  private static final double wheelDiam = 3.9;
  private static final double wheelCirc = (Math.PI * wheelDiam);
  private static final double TicksperRev = 1120;

  //Mechanism Variables:
  public static final double clawServoStartPosition = 0.0;
  public static final double holdServoStartPosition = 0.0;
  public static final double clawServoEndPosition = 1.0;
  public static final double holdServoEndPosition = 0.6;

  //Powers:
  public static final double slowPower = 0.4;
  public static final double mainPower = 0.6;
  public static final double fastPower = 0.8;

  //Objects:
  HardwareMap hardwareMap;
  public VuforiaImageInit imageInit = new VuforiaImageInit();
  public Yellow yellowDetector = new Yellow();

  //Detector Settings:
  public static final int detector[] = {0, 0, 0}, offset[] = {50, 50, 50};
  public static final String detectorName = "";
  public static final boolean turnOnFlash = true;
  public static final int zoomInit = 0;
  public static final int x = 0, y = 0, width = 0, height = 0;
  public static final int firstCount = 0, secondCount = 0, pixelCount = 0;
  public static final double resizeRatio = 0.1;

  /* INITIALIZATION AND CONTROL */

  //Initialization:
  public void init(HardwareMap hwMap, boolean type) {
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

    //Starts Vision:
    imageInit.initVuforia(hardwareMap, detector, detectorName, zoomInit, turnOnFlash);

    //IMU Init:
    BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
    imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    imu = hardwareMap.get(BNO055IMU.class, "imu");
    imu.initialize(imuParameters);

    //Encoders:
    leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    baseArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    //Checks the Case:
    if (type) {
      //Motors
      leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      baseArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

      //Zero Behavior:
      leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      baseArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    else {
      //Motors:
      leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      baseArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

      //Zero Behavior:
      leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
      leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
      rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
      rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
      baseArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //Directions:
    leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
    rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
    baseArmMotor.setDirection(DcMotor.Direction.FORWARD);

    //Inits the Servos:
    clawServo.setPosition(clawServoStartPosition);
    holdServo.setPosition(holdServoStartPosition);
  }

  /* VISION METHODS */

  //Gets the Position Pixel Count:
  public int getPixelsPosition() {
    //Position Variable (w/ Default):
    int position = 1;

    //Gets the Image, Bitmap RGB, and Count:
    Bitmap image = imageInit.getImage(resizeRatio);
    int rgb[][] = imageInit.getRGBArray(image, x, y, width, height);
    int booleanCount = imageInit.detectBooleanPixelCount(rgb, offset, pixelCount);

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

  //Calculates turn Rotations in Terms of Degrees:
  public double calculateTurnRotations(double angleDegrees) {
    final double angleInRadians = ((Math.PI / 180) * angleDegrees); //Converts Angles from Radians (for Arc Length)
    final double robotRotations = (robotDimensions / wheelCirc); //Width and Height in terms of Rotations
    final double d = Math.sqrt((robotRotations * robotRotations) + (robotRotations * robotRotations)); //Diameter Value of Circumscribed Robot
    final double r = d / 2; //Radius Value of Circumscribed Robot

    /* Ratio is Rotations/Degrees...So to find Rotations it is... x = (ratioRotations*correction)/ratioDegrees */

    //Rotations Needed to Correct Angle (arc length = r * theta):
    double driveRotations = r * angleInRadians;

    //Returns the Value:
    return driveRotations;
  }

  /* IMU Gyro Correction Method is Used in Conjunction with Turn Method, Reset Gyro Before Use:
     (PARAMS-expectedAngle should always be with direction [+ -> left and - -> right]) */
  public double[] getGyroCorrection(double expectedAngle, double resetValue) {
    //Main Array and Variables:
    double directionAndRotations[] = new double[2];
    double direction = 0; //Default Value

    //Gets IMU Reading:
    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    double imuReading = (double) angles.firstAngle;

    //Gets Correct IMU Reset Reading:
    double imuResetReading = imuReading + resetValue;

    //Sets the Direction:
    if (imuResetReading > expectedAngle) {
      direction = -1;
    }

    else if (imuResetReading < expectedAngle) {
      direction = 1;
    }

    directionAndRotations[0] = direction;

    //Calculates Correction Degrees:
    double correction = Math.abs(Math.abs(expectedAngle) - Math.abs(imuResetReading));

    /* Calculates Corrections in Rotations: */

    double driveRotations = calculateTurnRotations(correction);
    directionAndRotations[1] = driveRotations;

    //Returns the Direction (-1, 0, or 1) and the Rotations:
    return directionAndRotations;
  }

  //Resets Gyro Sensor Value:
  public double resetGyroValue() {
    //Gets Current Value:
    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    double currentIMUReading = (double) angles.firstAngle;

    //Resets Gyro Value:
    double resetValue = 0 - currentIMUReading;

    //Returns the Zero Value:
    return resetValue;
  }

  /* ROBOT MOVEMENT METHODS */

  //Resets Encoders After Every Movement:
  public void finishRun() {
    //Resets Power:
    leftFrontMotor.setPower(0);
    leftBackMotor.setPower(0);
    rightFrontMotor.setPower(0);
    rightBackMotor.setPower(0);
    baseArmMotor.setPower(0);

    //Resets the Encoders:
    leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    baseArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    //Runs Using Encoders:
    leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    baseArmMotor.setMode((DcMotor.RunMode.RUN_USING_ENCODER));
  }

  //Robot Move with Rotations:
  public void runRobot(String type, double distanceInRotations, double power) {
    int parts = (int) ((distanceInRotations * TicksperRev) * gearRatio);

    if (type.equalsIgnoreCase("forward")) {
      leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() + parts);
      leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() + parts);
      rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() + parts);
      rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() + parts);
    }

    else if (type.equalsIgnoreCase("backward")) {
      leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() - parts);
      leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() - parts);
      rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() - parts);
      rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() - parts);
    }

    leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    leftFrontMotor.setPower(power);
    leftBackMotor.setPower(power);
    rightFrontMotor.setPower(power);
    rightBackMotor.setPower(power);
  }

  //Robot Turn Motion with Rotations:
  public void turnRobot(String type, double distanceInRotations, double power) {
    int partsRotationsLeft = (int) ((distanceInRotations * TicksperRev) * gearRatio);
    int partsRotationsRight = (int) ((distanceInRotations * TicksperRev) * gearRatio);

    if (type.equalsIgnoreCase("left")) {
      leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() - partsRotationsLeft);
      leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() - partsRotationsLeft);
      rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() + partsRotationsRight);
      rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() + partsRotationsRight);
    }

    else if (type.equalsIgnoreCase("right")) {
      leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() + partsRotationsLeft);
      leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() + partsRotationsLeft);
      rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() - partsRotationsRight);
      rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() - partsRotationsRight);
    }

    leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    leftFrontMotor.setPower(power);
    leftBackMotor.setPower(power);
    rightFrontMotor.setPower(power);
    rightBackMotor.setPower(power);
  }

  //Robot Strafe Motion:
  public void shiftRobot(String type, double distanceInRotations, double power) {
    int partsRotationsLeft = (int) ((distanceInRotations * TicksperRev) * gearRatio);
    int partsRotationsRight = (int) ((distanceInRotations * TicksperRev) * gearRatio);

    if (type.equalsIgnoreCase("left")) {
      leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() - partsRotationsLeft);
      leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() + partsRotationsLeft);
      rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() + partsRotationsRight);
      rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() - partsRotationsRight);
    }

    else if (type.equalsIgnoreCase("right")) {
      leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() + partsRotationsLeft);
      leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() - partsRotationsLeft);
      rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() - partsRotationsRight);
      rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() + partsRotationsRight);
    }

    leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    leftFrontMotor.setPower(power);
    leftBackMotor.setPower(power);
    rightFrontMotor.setPower(power);
    rightBackMotor.setPower(power);
  }

  //Robot Diagonal Motion:
  public void diagRobot(String type, double distanceInRotations, double power) {
    int partsRotationsLeft = (int) ((distanceInRotations * TicksperRev) * gearRatio);
    int partsRotationsRight = (int) ((distanceInRotations * TicksperRev) * gearRatio);

    double LF = 0, LB = 0, RF = 0, RB = 0;

    if (type.equalsIgnoreCase("left and up")) {
      LF = 0.0;
      RF = power;
      RB = 0.0;
      LB = power;
      rightFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() + partsRotationsRight);
      leftBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() + partsRotationsLeft);
    }

    else if (type.equalsIgnoreCase("right and up")) {
      LF = power;
      RF = 0.0;
      RB = power;
      LB = 0.0;
      leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() + partsRotationsLeft);
      rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() + partsRotationsRight);
    }

    else if (type.equalsIgnoreCase("left and down")) {
      LF = power;
      RF = 0.0;
      RB = power;
      LB = 0.0;
      leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() - partsRotationsLeft);
      rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() - partsRotationsRight);
    }

    else if (type.equalsIgnoreCase("right and down")) {
      LF = 0.0;
      RF = power;
      RB = 0.0;
      LB = power;
      rightFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() - partsRotationsRight);
      leftBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() - partsRotationsLeft);
    }

    leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    leftFrontMotor.setPower(LF);
    leftBackMotor.setPower(LB);
    rightFrontMotor.setPower(RF);
    rightBackMotor.setPower(RB);
  }

  /* MECHANISM METHODS */

  //Move Arm:
  public void moveArm(String type, double distanceInRotations, double power) {
    int parts = (int) ((distanceInRotations * TicksperRev) * gearRatio);

    //Checks the Case;
    if (type.equalsIgnoreCase("up")) {
      baseArmMotor.setTargetPosition(baseArmMotor.getCurrentPosition() - parts);
    }

    else if (type.equalsIgnoreCase("down")) {
      baseArmMotor.setTargetPosition(baseArmMotor.getCurrentPosition() + parts);
    }

    baseArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    baseArmMotor.setPower(power);
  }

  //Operate Claw:
  public void operateClaw(String type) {
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