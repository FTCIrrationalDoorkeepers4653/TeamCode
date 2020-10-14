package org.firstinspires.ftc.teamcode.Systems.Core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Mechanisms extends PID {
  /* MECHANISMS SETUP VARIABLES */

  //Objects:
  static Robot robot = new Robot();

  //Mechanisms:
  public static DcMotor baseArmMotor;
  public static Servo clawServo;

  /* MECHANISMS MOVEMENT VARIABLES */

  //Mechanism Position Variables:
  public static double armDegrees = 80.0;
  public static boolean armUp = true;
  public static boolean clawClosed = true;

  //Mechanism Servo Variables:
  public static double clawOpenPosition = 1.0;
  public static double clawClosePosition = 0.0;

  /* MECHANISMS INITIALIZATION METHODS */

  //Initializes the Mechanisms:
  public static void initMechanisms() {
    /* Initialization */

    //Mechanism Maps:
    baseArmMotor = robot.hardwareMap.dcMotor.get("baseArmMotor");
    clawServo = robot.hardwareMap.servo.get("clawServo");

    /* Setup */

    //Mechanism Motors:
    baseArmMotor.setDirection(DcMotor.Direction.REVERSE);
    baseArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }

  /* MECHANISM AUTOMATION METHODS */

  //Automates the Movement of Arm:
  public void automateArm() {
    //Calculates the Time:
    int timeRequired = calculateTime(robot.getAngleRotations(armDegrees), robot.mainPower);

    //Moves the Arm:
    moveArm(robot.mainPower);
    completeCycle(timeRequired);

    //Opens the Claw:
    operateClaw();
  }

  /* MECHANISM MOVEMENT METHODS */

  //Mechanism Finish Run Method:
  public static void mechanismsFinishRun() {
    //Resets Power:
    baseArmMotor.setPower(robot.zeroPower);

    //Runs Using Encoder:
    baseArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }

  //Move Arm:
  public static void moveArm(double power) {
    //Checks the Case:
    if (armUp) {
      //Sets the Target Positions:
      baseArmMotor.setTargetPosition(baseArmMotor.getCurrentPosition() -
        robot.getParts(robot.getAngleRotations(armDegrees)));
      armUp = false;
    }

    else {
      //Sets the Target Positions:
      baseArmMotor.setTargetPosition(baseArmMotor.getCurrentPosition() +
        robot.getParts(robot.getAngleRotations(armDegrees)));
      armUp = true;
    }

    //Runs Motor to Position:
    baseArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    baseArmMotor.setPower(power);
  }

  //Operate Claw:
  public static void operateClaw() {
    //Checks the Case:
    if (clawClosed) {
      //Sets the Positions:
      clawServo.setPosition(clawClosePosition);
      clawClosed = false;
    }

    else {
      //Sets the Positions:
      clawServo.setPosition(clawOpenPosition);
      clawClosed = true;
    }
  }
}
