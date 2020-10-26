package org.firstinspires.ftc.teamcode.Programs.Driver;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.Core.GamePad;
import org.firstinspires.ftc.teamcode.Systems.Core.Robot;

@TeleOp(name = "DriverControl")
//@Disabled
public class DriverControl extends LinearOpMode {
  /* DRIVE VARIABLES */

  //Objects:
  private static Robot robot = new Robot();
  private GamePad driverPad;
  private GamePad operatorPad;

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
    driverPad = new GamePad(gamepad1);
    operatorPad = new GamePad(gamepad2);

    //Wait for Start:
    waitForStart();

    /* Run */

    //Loop Until Stop:
    mainLoop: while (opModeIsActive()) {
      //GamePad Method Calls:
      driverPad.setGamePad();
      operatorPad.setGamePad();

      //Robot Method Calls:
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
    double power = (driverPad.leftY * driverPad.leftY * driverPad.leftY);
    double turn = (driverPad.rightX * driverPad.rightX * driverPad.rightX);
    double left = (driverPad.leftTrigger * driverPad.leftTrigger * driverPad.leftTrigger);
    double right = (driverPad.rightTrigger * driverPad.rightTrigger * driverPad.rightTrigger);

    //Checks the Case:
    if (driverPad.leftBumper) {
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
    if (operatorPad.isAReleased()) {
      //Checks the Case:
      if (robot.mechanisms.arm == 0) {
        //Sets the Arm:
        robot.mechanisms.arm++;
      }

      else if (robot.mechanisms.arm == 1) {
        //Sets the Arm:
        robot.mechanisms.arm--;
      }

      //Moves the Arm:
      robot.mechanisms.automateArm(robot.mainPower);
    }
  }

  //Moves the Wobble Claw:
  public void moveClaw() {
    //Checks the Case:
    if (operatorPad.isYReleased()) {
      //Checks the Case:
      if (robot.mechanisms.claw == 0) {
        //Changes the Claw:
        robot.mechanisms.claw++;
      }

      else if (robot.mechanisms.claw == 1) {
        //Changes the Claw:
        robot.mechanisms.claw--;
      }

      //Operates the Claw:
      robot.mechanisms.operateClaw();
    }
  }
}