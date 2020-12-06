package org.firstinspires.ftc.teamcode.Systems.Core;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class Mechanisms extends Controller {
  /* MECHANISMS SETUP VARIABLES */

  //Objects:
  private static Robot robot = new Robot();

  //Motor Mechanisms:
  public static DcMotor baseArmMotor;
  public static DcMotor intakeMotor;
  public static DcMotorEx shooterMotor;

  //Servo Mechanisms:
  public static Servo clawServo;
  public static Servo shooterServo;
  public static Servo rampServo;
  public static Servo intakeServo;

  /* MECHANISMS ARM CONTROL VARIABLES */

  //Mechanism Arm Variables:
  public static double armDown = 200.0;
  public static int arm = 0;

  //Mechanism Claw Variables:
  public static double clawStartPosition = 0.2;
  public static double clawEndPosition = 0.6;
  public static int claw = 0;

  /* MECHANISM SHOOTER CONTROL VARIABLES */

  //Mechanism Shooter Variables:
  public static double shooterStartPosition = 0.9;
  public static double shooterEndPosition = 0.3;
  public static int shot = 0;

  //Mechanism Flywheel Variables:
  public static int shooter = 0;
  public static int shooterWait = 500;
  public static int shooterRevWait = (shooterWait * 2);
  public static double flywheelTicks = 28;
  public static double mainRPM = 3200.0;

  //Mechanism Ramp Variables:
  public static double rampStartPosition = 0.0;
  public static double rampEndPosition = 0.3;
  public static int ramp = 0;

  /* MECHANISM INTAKE CONTROL VARIABLES */

  //Mechanism Intake Variables:
  public static double intakeArmDown = 250.0;
  public static int intakeArm = 0;

  //Mechanism Intake Wheel Variables:
  public static int intakeWheel = 0;
  public static double intakeWheelStartPosition = 0.6;
  public static double intakeWheelEndPosition = 0.0;

  /* MECHANISMS INITIALIZATION METHODS */

  //Initializes the Mechanisms:
  public static void initMechanisms(HardwareMap hardwareMap) {
    /* Initialization */

    //Motor Mechanism Maps:
    baseArmMotor = hardwareMap.dcMotor.get("baseArmMotor");
    intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
    shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
    setupFlywheel(robot.uncoPower);

    //Servo Mechanism Maps:
    clawServo = hardwareMap.servo.get("clawServo");
    shooterServo = hardwareMap.servo.get("shooterServo");
    rampServo = hardwareMap.servo.get("rampServo");
    intakeServo = hardwareMap.servo.get("intakeServo");

    //Servo Mechanism Setup:
    clawServo.setPosition(clawStartPosition);
    shooterServo.setPosition(shooterStartPosition);
    rampServo.setPosition(rampStartPosition);
    intakeServo.setPosition(intakeWheelEndPosition);

    /* Setup */

    //Mechanism Motors Behavior:
    baseArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    //Mechanism Motors Direction:
    baseArmMotor.setDirection(DcMotor.Direction.REVERSE);
    intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    shooterMotor.setDirection(DcMotor.Direction.REVERSE);

    //Mechanism Motors Encoders Reset:
    baseArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    //Mechanism Motors Encoders:
    baseArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    //Mechanism Motors Powers:
    baseArmMotor.setPower(robot.zeroPower);
    intakeMotor.setPower(robot.zeroPower);
    shooterMotor.setPower(robot.zeroPower);

    /* Variables */

    //Resetting Positions:
    arm = 0;
    claw = 0;
    shooter = 0;
    shot = 0;
    intakeArm = 0;
    intakeWheel = 0;
  }

  /* MECHANISM AUTOMATION METHODS */

  //Automate Intake Method:
  public void automateIntake(double power) {
    //Time Calculation:
    double rotations = robot.getAngleRotations(intakeArmDown);
    int time = calculateTime(rotations, power);

    //Checks the Case:
    if (intakeArm == 0) {
      //Sets the Intake Arm:
      intakeArm++;
    }

    else {
      //Sets the Intake Arm:
      intakeArm--;
    }

    //Runs the Intake Arm:
    operateIntakeArm(power);
    completeCycle(time);

    //Sets the Intake Wheel:
    intakeWheel = 0;
    operateIntakeWheel();
  }

  //Automate Intake Wheel Method:
  public void automateIntakeWheel() {
    //Checks the Case:
    if (intakeWheel == 0) {
      //Sets the Intake Wheel:
      intakeWheel++;
      operateIntakeWheel();
    }

    else {
      //Sets the Intake Wheel:
      intakeWheel--;
      operateIntakeWheel();
    }
  }

  //Automate Arm Method:
  public void automateArm(double power) {
    //Time Calculation:
    double rotations = robot.getAngleRotations(armDown);
    int time = calculateTime(rotations, power);

    //Checks the Case:
    if (arm == 0) {
      //Operates the Arm:
      arm++;
      operateArm(power);
      completeCycle(time);
    }

    else  {
      //Operates the Arm:
      arm--;
      operateArm(power);
      completeCycle(time);

      //Sets the Claw Position:
      claw = 0;
      operateClaw();
    }
  }

  //Automate Claw Method:
  public void automateClaw() {
    //Checks the Case:
    if (claw == 0) {
      //Sets the Claw:
      claw++;
      operateClaw();
    }

    else {
      //Sets the Claw:
      claw--;
      operateClaw();
    }
  }

  //Automate Shooter Method:
  public void automateShooter() {
    //Shoots Ring (Flywheel Must Be On):
    ramp = 0;
    shot = 1;
    operateRamp();
    operateShooter();
    completeCycle(shooterWait);

    //Resets the Shooter:
    shot = 0;
    operateRamp();
    operateShooter();
  }

  //Automate Flywheel Method:
  public void automateFlywheel() {
    //Checks the Case:
    if (shooter == 0) {
      //Sets the Shooter:
      shooter++;
      operateFlywheel();
    }

    else {
      //Sets the Shooter:
      shooter--;
      operateFlywheel();
    }
  }

  /* MECHANISM MOTOR OPERATION METHODS */

  //Mechanism Finish Run Method:
  public static void mechanismsFinishRun() {
    //Resets Powers:
    baseArmMotor.setPower(robot.zeroPower);
    intakeMotor.setPower(robot.zeroPower);

    //Resets the Encoders:
    baseArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    //Sets Encoders:
    baseArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }

  //Operates the Flywheel:
  public static void operateFlywheel() {
    //Checks the Case:
    if (shooter == 0) {
      //Sets the Motor:
      double ticks = calculateTicks(robot.zeroPower);
      applyControlMotorPowerEx(shooterMotor, ticks);
    }

    else if (shooter == 1) {
      //Sets the Motor:
      double ticks = calculateTicks(mainRPM);
      applyControlMotorPowerEx(shooterMotor, ticks);
    }
  }

  //Operates the Arm:
  public static void operateArm(double power) {
    //Target Variable:
    int endTarget = robot.getParts(robot.getAngleRotations(armDown));

    //Checks the Case:
    if (arm == 0) {
      //Runs the Target Positions:
      baseArmMotor.setTargetPosition(baseArmMotor.getCurrentPosition() - endTarget);
      baseArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      applyControlMotorPower(baseArmMotor, power);
    }

    else if (arm == 1) {
      //Runs the Target Positions:
      baseArmMotor.setTargetPosition(baseArmMotor.getCurrentPosition() + endTarget);
      baseArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      applyControlMotorPower(baseArmMotor, power);
    }
  }

  //Operates the Intake Arm:
  public static void operateIntakeArm(double power) {
    //Target Variable:
    int endTarget = robot.getParts(robot.getAngleRotations(intakeArmDown));

    //Checks the Case:
    if (intakeArm == 0) {
      //Runs the Target Positions:
      intakeMotor.setTargetPosition(intakeMotor.getCurrentPosition() - endTarget);
      intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      applyControlMotorPower(intakeMotor, power);
    }

    else if (intakeArm == 1) {
      //Runs the Target Positions:
      intakeMotor.setTargetPosition(intakeMotor.getCurrentPosition() + endTarget);
      intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      applyControlMotorPower(intakeMotor, power);
    }
  }

  /* MECHANISMS SERVO OPERATION METHODS */

  //Operate Intake Wheel:
  public static void operateIntakeWheel() {
    //Checks the Case:
    if (intakeWheel == 0) {
      //Sets the Servo:
      intakeServo.setPosition(intakeWheelStartPosition);
    }

    else if (intakeWheel == 1) {
      //Sets the Servo:
      intakeServo.setPosition(intakeWheelEndPosition);
    }
  }

  //Operate Ramp:
  public static void operateRamp() {
    //Checks the Case:
    if (ramp == 0) {
      //Sets the Servo:
      rampServo.setPosition(rampStartPosition);
    }

    else if (ramp == 1) {
      //Sets the Servo:
      rampServo.setPosition(rampEndPosition);
    }
  }

  //Operate Shooter:
  public static void operateShooter() {
    //Checks the Case:
    if (shot == 0) {
      //Sets the Servo:
      shooterServo.setPosition(shooterStartPosition);
    }

    else if (shot == 1) {
      //Sets the Servo:
      shooterServo.setPosition(shooterEndPosition);
    }
  }

  //Operate Claw:
  public static void operateClaw() {
    //Checks the Case:
    if (claw == 0) {
      //Sets the Servo:
      clawServo.setPosition(clawStartPosition);
    }

    else if (claw == 1) {
      //Sets the Servo:
      clawServo.setPosition(clawEndPosition);
    }
  }

  /* MECHANISMS UTILITY METHODS */

  //Flywheel Initialization Method:
  public static void setupFlywheel(double maxRPMValue) {
    //Sets the Motor Configuration Type:
    MotorConfigurationType motorConfigurationType = shooterMotor.getMotorType().clone();
    motorConfigurationType.setAchieveableMaxRPMFraction(maxRPMValue);
    shooterMotor.setMotorType(motorConfigurationType);
  }

  //Calculate Ticks from RPM Method:
  public static double calculateTicks(double RPM) {
    //Calculates and Returns the Values:
    double RPS = (RPM / 60.0);
    double TPS = (RPS * flywheelTicks);
    return TPS;
  }
}
