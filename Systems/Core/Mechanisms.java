package org.firstinspires.ftc.teamcode.Systems.Core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Mechanisms extends Controller {
  /* MECHANISMS SETUP VARIABLES */

  //Objects:
  private static Robot robot = new Robot();

  //Mechanisms:
  public static DcMotor baseArmMotor;
  public static Servo clawServo;

  /* MECHANISMS CONTROL VARIABLES */

  //Mechanism Arm Variables:
  public static double armDown = 170.0;
  public static int arm = 0;

  //Mechanism Claw Variables:
  public static double clawStartPosition = 0.0;
  public static double clawEndPosition = 0.6;
  public static int claw = 0;

  /* MECHANISMS INITIALIZATION METHODS */

  //Initializes the Mechanisms:
  public static void initMechanisms() {
    /* Initialization */

    //Mechanism Maps:
    baseArmMotor = robot.hardwareMap.dcMotor.get("baseArmMotor");
    clawServo = robot.hardwareMap.servo.get("clawServo");
    clawServo.setPosition(clawStartPosition);

    /* Setup */

    //Mechanism Motors:
    baseArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    baseArmMotor.setDirection(DcMotor.Direction.REVERSE);
    baseArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    baseArmMotor.setPower(robot.zeroPower);

    /* Variables */

    //Resetting Positions:
    arm = 0;
    claw = 0;
  }

  /* MECHANISM AUTOMATION METHODS */

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

  /* MECHANISM OPERATION METHODS */

  //Mechanism Finish Run Method:
  public static void mechanismsFinishRun() {
    //Resets Power and Resets Encoders:
    baseArmMotor.setPower(robot.zeroPower);
    baseArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
      applyControlArmPower(power);
    }

    else if (arm == 1) {
      //Runs the Target Positions:
      baseArmMotor.setTargetPosition(endTarget);
      baseArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      applyControlArmPower(power);
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
