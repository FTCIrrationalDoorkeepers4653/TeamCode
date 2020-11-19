package org.firstinspires.ftc.teamcode.Systems.Core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Mechanisms extends Controller {
  /* MECHANISMS SETUP VARIABLES */

  //Objects:
  private static Robot robot = new Robot();

  //Motor Mechanisms:
  public static DcMotor baseArmMotor;
  public static DcMotor intakeMotor;
  public static DcMotor shooterMotor;

  //Servo Mechanisms:
  public static Servo clawServo;
  public static Servo shooterServo;
  public static Servo rampServo;
  public static Servo intakeServo;

  /* MECHANISMS CONTROL VARIABLES */

  //Mechanism Arm Variables:
  public static double armDown = 175.0;
  public static int arm = 0;

  //Mechanism Claw Variables:
  public static double clawStartPosition = 0.0;
  public static double clawEndPosition = 0.6;
  public static int claw = 0;

  //Mechanism Shooter Variables:
  public static double shooterStartPosition = 0.8;
  public static double shooterEndPosition = 0.5;
  public static int shot = 0;

  //Mechanism Flywheel Variables:
  public static int shooter = 0;
  public static int ringCount = 3;
  public static int shooterWait = 500;

  //Mechanism Ramp Variables:
  public static double rampStartPosition = 0.0;
  public static double rampEndPosition = 0.3;
  public static int ramp = 0;

  //Mechanism Intake Arm Variables:
  public static double intakeArmDown = 150.0;
  public static int intakeArm = 0;

  //Mechanism Intake Claw Variables:
  public static double intakeStartPosition = 0.0;
  public static double intakeEndPosition = 0.6;
  public static double intakeClaw = 0;

  /* MECHANISMS INITIALIZATION METHODS */

  //Initializes the Mechanisms:
  public static void initMechanisms(HardwareMap hardwareMap) {
    /* Initialization */

    //Motor Mechanism Maps:
    baseArmMotor = hardwareMap.dcMotor.get("baseArmMotor");
    intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
    shooterMotor = hardwareMap.dcMotor.get("shooterMotor");

    //Servo Mechanism Maps:
    clawServo = hardwareMap.servo.get("clawServo");
    shooterServo = hardwareMap.servo.get("shooterServo");
    rampServo = hardwareMap.servo.get("rampServo");
    intakeServo = hardwareMap.servo.get("intakeServo");

    //Servo Mechanism Positions:
    clawServo.setPosition(clawStartPosition);
    shooterServo.setPosition(shooterStartPosition);
    rampServo.setPosition(rampStartPosition);
    intakeServo.setPosition(intakeStartPosition);

    /* Setup */

    //Mechanism Motors Behavior:
    baseArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    //Mechanism Motors Direction:
    baseArmMotor.setDirection(DcMotor.Direction.REVERSE);
    intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    shooterMotor.setDirection(DcMotor.Direction.REVERSE);

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
    intakeClaw = 0;
  }

  /* MECHANISM AUTOMATION METHODS */

  //Automate Intake Method:
  public void automateIntake(double power) {
    //Checks the Case:
    if (ringCount < 4) {
      //Intake Arm With Time:
      int time = calculateTime(robot.getAngleRotations(intakeArmDown), power);
      operateIntakeArm(power);
      completeCycle(time);

      //Opens the Intake Claw:
      intakeClaw = 0;
      operateIntakeClaw();
      ringCount++;
    }
  }

  //Automate Arm Method:
  public void automateArm(double power) {
    //Runs the Arm With Time:
    int time = calculateTime(robot.getAngleRotations(armDown), power);
    operateArm(power);
    completeCycle(time);

    //Checks the Case:
    if (arm == 1) {
      //Sets the Claw Position:
      claw = 0;
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

    //Checks the Case:
    if (ringCount > 0 && shooter == 1) {
      //Sets the Ring Count:
      ringCount--;
    }
  }

  //Automate Flywheel Method:
  public void automateFlywheel(double power) {
    //Checks the Case:
    if (ringCount > 0 && ringCount < 4) {
      //Operates Flywheel:
      shooter = 1;
      operateFlywheel(power);
    }

    else {
      //Operates the Flywheel:
      shooter = 0;
      operateFlywheel(power);
    }
  }

  /* MECHANISM MOTOR OPERATION METHODS */

  //Mechanism Finish Run Method:
  public static void mechanismsFinishRun() {
    //Resets Powers:
    baseArmMotor.setPower(robot.zeroPower);
    intakeMotor.setPower(robot.zeroPower);

    //Resets Encoders:
    baseArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }

  //Operates the Flywheel:
  public static void operateFlywheel(double power) {
    //Checks the Case:
    if (shooter == 0) {
      //Sets the Motor Power:
      applyControlMotorPower(shooterMotor, robot.zeroPower);
    }

    else if (shooter == 1) {
      //Sets the Motor Power:
      applyControlMotorPower(shooterMotor, power);
    }
  }

  //Operates the Arm:
  public static void operateArm(double power) {
    //Target Variables:
    int startTarget = 0;
    int endTarget = robot.getParts(robot.getAngleRotations(armDown));

    //Checks the Case:
    if (arm == 0) {
      //Runs the Target Positions:
      baseArmMotor.setTargetPosition(startTarget);
      baseArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      applyControlMotorPower(baseArmMotor, power);
    }

    else if (arm == 1) {
      //Runs the Target Positions:
      baseArmMotor.setTargetPosition(endTarget);
      baseArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      applyControlMotorPower(baseArmMotor, power);
    }
  }

  //Operates the Intake Arm:
  public static void operateIntakeArm(double power) {
    //Target Variables:
    int startTarget = 0;
    int endTarget = robot.getParts(robot.getAngleRotations(intakeArmDown));

    //Checks the Case:
    if (intakeArm == 0) {
      //Runs the Target Positions:
      intakeMotor.setTargetPosition(startTarget);
      intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      applyControlMotorPower(intakeMotor, power);
    }

    else if (intakeArm == 1) {
      //Runs the Target Positions:
      intakeMotor.setTargetPosition(endTarget);
      intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      applyControlMotorPower(intakeMotor, power);
    }
  }

  /* MECHANISMS SERVO OPERATION METHODS */

  //Operate Intake Claw:
  public static void operateIntakeClaw() {
    //Checks the Case:
    if (intakeClaw == 0) {
      //Sets the Servo:
      intakeServo.setPosition(intakeStartPosition);
    }

    else if (intakeClaw == 1) {
      //Sets the Servo:
      intakeServo.setPosition(intakeEndPosition);
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
}
