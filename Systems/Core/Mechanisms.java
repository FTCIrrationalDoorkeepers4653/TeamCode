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

  //Robot Object:
  private static Robot robot = new Robot();

  //Motor Mechanisms:
  public static DcMotor baseArmMotor;
  public static DcMotor intakeMotor;
  public static DcMotorEx shooterMotor;

  //Servo Mechanisms:
  public static Servo clawServo;
  public static Servo shooterServo;

  /* MECHANISMS ARM CONTROL VARIABLES */

  //Mechanism Arm Variables:
  public static double armDown = 180.0;
  public static int arm = 0;

  //Mechanism Claw Variables:
  public static double clawStartPosition = 0.2;
  public static double clawEndPosition = 1.0;
  public static int claw = 0;

  /* MECHANISM SHOOTER CONTROL VARIABLES */

  //Mechanism Shooter Variables:
  public static double shooterStartPosition = 0.9;
  public static double shooterEndPosition = 0.6;
  public static int shot = 0;
  public static int shooterWait = 500;

  //Mechanism Flywheel Variables:
  public static int shooter = 0;
  public static double flywheelTicks = 28;
  public static double mainRPM = 3275.0;

  /* MECHANISM INTAKE CONTROL VARIABLES */

  //Mechanism Intake Variables:
  public static int intake = 0;

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

    //Servo Mechanism Setup:
    clawServo.setPosition(clawStartPosition);
    shooterServo.setPosition(shooterStartPosition);

    /* Setup */

    //Mechanism Motors Behavior:
    baseArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
    intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    //Mechanism Motors Powers:
    baseArmMotor.setPower(robot.zeroPower);
    intakeMotor.setPower(robot.zeroPower);
    shooterMotor.setPower(robot.zeroPower);

    /* Variables */

    //Resetting Positions:
    claw = 0;
    arm = 0;
    shooter = 0;
    shot = 0;
    intake = 0;
  }

  /* MECHANISM AUTOMATION METHODS */

  //Automate Intake Method:
  public void automateIntake(double power) {
    //Checks the Case:
    if (intake == 0) {
      //Sets the Intake Arm:
      intake++;
      operateIntake(power);
    }

    else {
      //Sets the Intake Arm:
      intake--;
      operateIntake(power);
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
    shot = 1;
    operateShooter();
    completeCycle(shooterWait);

    //Resets the Shooter:
    shot = 0;
    operateShooter();
    completeCycle(shooterWait);
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

  /* MECHANISM OPERATION METHODS */

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
      shooterMotor.setVelocity(ticks);
    }

    else if (shooter == 1) {
      //Sets the Motor:
      double ticks = calculateTicks(mainRPM);
      shooterMotor.setVelocity(ticks);
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
      baseArmMotor.setPower(power);
    }

    else if (arm == 1) {
      //Runs the Target Positions:
      baseArmMotor.setTargetPosition(baseArmMotor.getCurrentPosition() + endTarget);
      baseArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      baseArmMotor.setPower(power);
    }
  }

  //Operates the Intake Arm:
  public static void operateIntake(double power) {
    //Checks the Case:
    if (intake == 0) {
      //Runs the Intake:
      intakeMotor.setPower(power);
    }

    else if (intake == 1) {
      //Runs the Target Positions:
      intakeMotor.setPower(robot.zeroPower);
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
