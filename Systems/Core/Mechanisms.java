package org.firstinspires.ftc.teamcode.Systems.Core;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Mechanisms extends Controller {
  /* MECHANISMS SETUP VARIABLES */

  //Objects:
  static Robot robot = new Robot();

  //Mechanisms:
  public static DcMotor baseArmMotor;
  public static CRServo clawServo;

  /* MECHANISMS MOVEMENT VARIABLES */

  //Mechanism Degree Variables:
  public static double armDown = 90.0;
  public static double armMiddle = 45.0;

  //Mechanism Position Variables:
  public static int claw = 0;
  public static int arm = 0;

  /* MECHANISMS INITIALIZATION METHODS */

  //Initializes the Mechanisms:
  public static void initMechanisms() {
    /* Initialization */

    //Mechanism Maps:
    baseArmMotor = robot.hardwareMap.dcMotor.get("baseArmMotor");
    clawServo = robot.hardwareMap.crservo.get("clawServo");

    /* Setup */

    //Mechanism Motors:
    baseArmMotor.setPower(robot.zeroPower);
    baseArmMotor.setDirection(DcMotor.Direction.REVERSE);
    baseArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    baseArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    //Mechanism Servos:
    clawServo.setPower(robot.zeroPower);
    clawServo.setDirection(CRServo.Direction.REVERSE);
  }

  /* MECHANISM MOTOR MOVEMENT METHODS */

  //Mechanism Finish Run Method:
  public static void mechanismsFinishRun() {
    //Resets Power:
    baseArmMotor.setPower(robot.zeroPower);

    //Runs Using Encoder:
    baseArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }

  //Move Arm:
  public static void operateArm(double power) {
    //Target Positions:
    int startTarget = 0;
    int middleTarget = robot.getParts(robot.getAngleRotations(armMiddle));
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
      baseArmMotor.setTargetPosition(middleTarget);
      baseArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      applyControlArmPower(power);
    }

    else if (arm == 2) {
      //Runs the Target Positions:
      baseArmMotor.setTargetPosition(endTarget);
      baseArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      applyControlArmPower(power);
    }
  }

  /* MECHANISM SERVO MOVEMENT METHODS */

  //Operate Claw:
  public static void operateClaw(double power) {
    //Checks the Case:
    if (claw == 0) {
      //Sets the Servo:
      clawServo.setPower(robot.zeroPower);
    }

    else if (claw == 1) {
      //Sets the Servo:
      clawServo.setPower(power);
    }

    else if (claw == -1) {
      //Sets the Servo:
      clawServo.setPower(-power);
    }
  }
}
