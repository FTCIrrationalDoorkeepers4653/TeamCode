package org.firstinspires.ftc.teamcode.Programs.Driver;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Systems.Core.Robot;

@TeleOp(name = "DriverControl")
//@Disabled
public class DriverControl extends LinearOpMode {
  /* DRIVE VARIABLES */

  //Objects:
  static Robot robot = new Robot();

  /* OPMODE METHODS */

  //RunOpMode Method:
  @Override
  public void runOpMode() {
    /* Initialization */

    //Sets Status:
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    //Initialize Robot:
    robot.init(hardwareMap, false, false);
    robot.mechanisms.initMechanisms();

    //Wait for Start:
    waitForStart();

    /* Run */

    //Loop Until Stop:
    mainLoop: while (opModeIsActive()) {
      //Method Calls:
      moveRobot();
      moveArm();
      moveClaw();

      //Sets Status:
      telemetry.addData("Status", "Running");
      telemetry.update();
    }

    /* Stop */

    //Sets Status:
    telemetry.addData("Status", "Stopped");
    telemetry.update();
  }

  /* DRIVER CONTROL METHODS */

  //Moves the Robot:
  public void moveRobot() {
    //Gets the Gamepad Control Values:
    double power = (gamepad1.left_stick_y * gamepad1.left_stick_y * gamepad1.left_stick_y);
    double turn = (gamepad1.right_stick_x * gamepad1.right_stick_x * gamepad1.right_stick_x);
    double left = (gamepad1.left_trigger * gamepad1.left_trigger * gamepad1.left_trigger);
    double right = (gamepad1.right_trigger * gamepad1.right_trigger * gamepad1.right_trigger);

    //Checks the Case:
    if (gamepad1.left_bumper) {
      //Adjusts Slow Mode Speed:
      power /= robot.speedControl;
      turn /= robot.speedControl;
      left /= robot.speedControl;
      right /= robot.speedControl;
    }

    //Checks the Case:
    if (power != robot.zeroPower) {
      //Sets the Motor Power:
      robot.leftFrontMotor.setPower(power);
      robot.leftBackMotor.setPower(power);
      robot.rightFrontMotor.setPower(power);
      robot.rightBackMotor.setPower(power);
    }

    else if (turn != robot.zeroPower) {
      //Sets the Motor Power:
      robot.leftFrontMotor.setPower(-turn);
      robot.leftBackMotor.setPower(-turn);
      robot.rightFrontMotor.setPower(turn);
      robot.rightBackMotor.setPower(turn);
    }

    else if (left != robot.zeroPower) {
      //Sets the Motor Power:
      robot.leftFrontMotor.setPower(left);
      robot.leftBackMotor.setPower(-left);
      robot.rightFrontMotor.setPower(-left);
      robot.rightBackMotor.setPower(left);
    }

    else if (right != robot.zeroPower) {
      //Sets the Motor Power:
      robot.leftFrontMotor.setPower(-right);
      robot.leftBackMotor.setPower(right);
      robot.rightFrontMotor.setPower(right);
      robot.rightBackMotor.setPower(-right);
    }

    else {
      //Sets the Motor Power:
      robot.leftFrontMotor.setPower(robot.zeroPower);
      robot.leftBackMotor.setPower(robot.zeroPower);
      robot.rightFrontMotor.setPower(robot.zeroPower);
      robot.rightBackMotor.setPower(robot.zeroPower);
    }
  }

  //Moves the Wobble Arm:
  public void moveArm() {
    //Checks the Case:
    if (gamepad1.a) {
      //Checks the Case:
      if (robot.mechanisms.arm == 0) {
        //Sets the Arm:
        robot.mechanisms.arm++;
      }

      else if (robot.mechanisms.arm == 1) {
        //Sets the Arm:
        robot.mechanisms.arm++;
      }

      else if (robot.mechanisms.arm == 2) {
        //Sets the Arm:
        robot.mechanisms.arm -= 2;
      }

      //Moves the Arm:
      robot.mechanisms.operateArm(robot.mainPower);
      robot.mechanisms.completeCycle();
    }
  }

  //Moves the Wobble Claw:
  public void moveClaw() {
    //Checks the Case:
    if (gamepad1.y) {
      //Checks the Case:
      if (robot.mechanisms.claw == 0) {
        //Changes the Claw:
        robot.mechanisms.claw++;
      }

      else if (robot.mechanisms.claw == 1) {
        //Changes the Claw:
        robot.mechanisms.claw -= 2;
      }

      else if (robot.mechanisms.claw == -1) {
        //Changes the Claw:
        robot.mechanisms.claw++;
      }
    }

    //Operates the Claw:
    robot.mechanisms.operateClaw();
  }
}