package org.firstinspires.ftc.teamcode.Systems.Core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class Mechanisms extends Positions {
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
  public static Servo parkServo;

  /* MECHANISMS ARM CONTROL VARIABLES */

  //Mechanism Arm Variables:
  public static double fullArmDown = 260.0;
  public static double armDown = 220.0;
  public static double armMid = (armDown / 3.5);
  public static double armSecond = (armDown - armMid);
  public static int arm = 0;

  //Mechanism Claw Variables:
  public static double clawStartPosition = 0.0;
  public static double clawTeleStartPosition = 0.4;
  public static double clawEndPosition = 1.0;
  public static int claw = 0;

  /* MECHANISM SHOOTER CONTROL VARIABLES */

  //Mechanism Shooter Variables:
  public static double shooterStartPosition = 0.9;
  public static double shooterEndPosition = 0.5;
  public static int shooterWait = 500;
  public static int shot = 0;
  public static int intake = 0;

  //Mechanism Flywheel Variables:
  public static int shooter = 0;
  public static double flywheelTicks = 28;
  public static double mainRPM = 3800.0;
  public static double autoRPM = 3820.0;

  /* MECHANISM PARK CONTROL VARIABLES */

  //Mechanism Park Servo Variables:
  public static int park = 0;
  public static double parkStartPosition = 0.5;
  public static double parkEndPosition = 1.0;

  /* MECHANISMS INITIALIZATION METHODS */

  //Constructor:
  public Mechanisms() {
    super();
  }

  //Initializes the Mechanisms:
  public static void initMechanisms(HardwareMap hardwareMap, boolean auto) {
    /* Initialization */

    //Motor Mechanism Maps:
    baseArmMotor = hardwareMap.dcMotor.get("baseArmMotor");
    intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
    shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
    setupFlywheel(robot.uncoPower);

    //Servo Mechanism Maps:
    clawServo = hardwareMap.servo.get("clawServo");
    shooterServo = hardwareMap.servo.get("shooterServo");
    parkServo = hardwareMap.servo.get("parkServo");

    //Checks the Case:
    if (auto) {
      //Servo Mechanism Setup:
      clawServo.setPosition(clawEndPosition);
      shooterServo.setPosition(shooterStartPosition);
      parkServo.setPosition(parkStartPosition);
    }

    else {
      //Servo Mechanism Setup:
      clawServo.setPosition(clawStartPosition);
      shooterServo.setPosition(shooterStartPosition);
      parkServo.setPosition(parkStartPosition);
    }

    /* Setup */

    //Mechanism Motors Behavior:
    baseArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    //Mechanism Motors Direction:
    baseArmMotor.setDirection(DcMotor.Direction.REVERSE);
    intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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
  }

  //Custom Values Initialization Method:
  public static void initCustomValues(int values[]) {
    //Checks the Case:
    if (values.length == 6) {
      //Sets the Values:
      claw = values[0];
      arm = values[1];
      shooter = values[2];
      shot = values[3];
      intake = values[4];
      park = values[5];
    }
  }

  /* MECHANISM AUTOMATION METHODS */

  //Automate Intake Method:
  public void automateIntake() {
    //Checks the Case:
    if (intake == 0) {
      //Sets the Intake:
      intake++;
    }

    else {
      //Sets the Intake:
      intake--;
    }

    //Operates the Intake:
    operateIntake();
  }

  //Automate Arm Method:
  public void automateArm() {
    //Checks the Case:
    if (arm == 0) {
      //Operates the Arm:
      arm++;
      operateArm();
    }

    else if (arm == 1) {
      //Operates the Arm:
      arm++;
      operateArm();
    }

    else {
      //Operates the Arm:
      arm -= 2;
      operateArm();
    }
  }

  //Automate Claw Method:
  public void automateClaw(boolean teleOp) {
    //Checks the Case:
    if (!teleOp) {
      //Checks the Case:
      if (claw == 0) {
        //Sets the Claw:
        claw++;
        operateClaw();
      } else {
        //Sets the Claw:
        claw--;
        operateClaw();
      }
    }

    else {
      //Checks the Case:
      if (claw == 0) {
        //Sets the Claw:
        claw++;
        operateClawTele();
      }

      else {
        //Sets the Claw:
        claw--;
        operateClawTele();
      }
    }
  }

  //Automate Shooter Method:
  public void automateShooter(int start) {
    //Waits for the Ring to Shoot:
    sleep(start);

    //Shoots Ring:
    shot = 1;
    operateShooter();

    //Waits for Ring to Reset:
    sleep(shooterWait);

    //Resets the Shooter:
    shot = 0;
    operateShooter();
  }

  //Automate Flywheel Method:
  public void automateFlywheel(boolean auto) {
    //Checks the Case:
    if (shooter == 0) {
      //Sets the Shooter:
      shooter++;

      //Checks the Case:
      if (auto) {
        //Sets the Flywheel:
        operateFlywheel(autoRPM);
      }

      else {
        //Sets the Flywheel:
        operateFlywheel(mainRPM);
      }
    }

    else {
      //Sets the Shooter:
      shooter--;
      operateFlywheel(robot.zeroPower);
    }
  }

  //Automate Custom Flywheel Method:
  public void automateCustomFlywheel(double RPM) {
    //Checks the Case:
    if (shooter == 0) {
      //Sets the Shooter:
      shooter++;
      operateFlywheel(RPM);
    }

    else {
      //Sets the Shooter:
      shooter--;
      operateFlywheel(robot.zeroPower);
    }
  }

  //Automate Park Method:
  public void automatePark() {
    //Checks the Case:
    if (park == 0) {
      //Sets the Position:
      park++;
      operatePark();
    }

    else {
      //Sets the Position:
      park--;
      operatePark();
    }
  }

  /* MECHANISM OPERATION METHODS */

  //Operates the Flywheel:
  public void operateFlywheel(double RPM) {
    //Checks the Case:
    if (shooter == 0) {
      //Sets the Motor:
      double ticks = calculateTicks(robot.zeroPower);
      shooterMotor.setVelocity(ticks);
    }

    else if (shooter == 1) {
      //Sets the Motor:
      double ticks = calculateTicks(RPM);
      shooterMotor.setVelocity(ticks);
    }
  }

  //Operates the Arm:
  public void operateArm() {
    //Rotation Variables:
    double endRotations = robot.getAngleRotations(fullArmDown);
    double midRotations = robot.getAngleRotations(armMid);
    double secondRotations = robot.getAngleRotations(armSecond);

    //Target Variables:
    int endTarget = robot.getParts(endRotations);
    int midTarget = robot.getParts(midRotations);
    int secondTarget = robot.getParts(secondRotations);

    //Checks the Case:
    if (arm == 0) {
      //Runs Motor to Position:
      baseArmMotor.setTargetPosition(baseArmMotor.getCurrentPosition() + endTarget);
      baseArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      baseArmMotor.setPower(robot.slowPower);
      sleep(robot.calculateTime(endRotations, robot.movePower));
    }

    else if (arm == 1) {
      //Runs Motor to Position:
      baseArmMotor.setTargetPosition(baseArmMotor.getCurrentPosition() - midTarget);
      baseArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      baseArmMotor.setPower(robot.slowPower);
      sleep(robot.calculateTime(midRotations, robot.slowPower));
    }

    else if (arm == 2) {
      //Runs Motor to Position:
      baseArmMotor.setTargetPosition(baseArmMotor.getCurrentPosition() - secondTarget);
      baseArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      baseArmMotor.setPower(robot.slowPower);
      sleep(robot.calculateTime(secondRotations, robot.slowPower));
    }

    //Resets Motor:
    baseArmMotor.setPower(robot.zeroPower);
    baseArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    baseArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }

  //Operates the Intake Arm:
  public void operateIntake() {
    //Checks the Case:
    if (intake == 0) {
      //Runs the Intake:
      intakeMotor.setPower(robot.zeroPower);
    }

    else if (intake == 1) {
      //Runs the Intake:
      intakeMotor.setPower(-robot.uncoPower);
    }
  }

  //Operate Shooter:
  public void operateShooter() {
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
  public void operateClaw() {
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

  //Operate Claw TeleOp:
  public void operateClawTele() {
    //Checks the Case:
    if (claw == 0) {
      //Sets the Servo:
      clawServo.setPosition(clawTeleStartPosition);
    }

    else if (claw == 1) {
      //Sets the Servo:
      clawServo.setPosition(clawEndPosition);
    }
  }

  //Operate Park:
  public void operatePark() {
    //Checks the Case:
    if (park == 0) {
      //Sets the Position:
      parkServo.setPosition(parkStartPosition);
    }

    else if (park == 1) {
      //Sets the Position:
      parkServo.setPosition(parkEndPosition);
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
