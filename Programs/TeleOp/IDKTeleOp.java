package org.firstinspires.ftc.teamcode.Programs.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Systems.Movement.IDKRobot;

@TeleOp(name = "IDKTeleOp")
//@Disabled
public class IDKTeleOp extends OpMode {
  /* TELEOP VARIABLES */

  //Runtime Object:
  private ElapsedTime runtime = new ElapsedTime();
  IDKRobot robot = new IDKRobot();

  //Motor Variables:
  boolean runType = false;
  double clawClosed = -1;

  /* TELEOP METHODS */

  //Runs on INIT:
  @Override
  public void init() {
    //Init Robot:
    robot.init(hardwareMap, runType);

    //Sets Status:
    telemetry.addData("Status", "Initialized");
    telemetry.update();
  }

  //Runs on START:
  @Override
  public void loop() {
    //Movement Method Calls:
    moveRobot();
    moveArm();
    grabWobble();

    //Sets Status:
    telemetry.addData("Status", "Running");
    telemetry.update();
  }

  //Runs on STOP:
  @Override
  public void stop() {
    //Sets Status:
    telemetry.addData("Status", "Stopped");
    telemetry.update();
  }

  /* CONTROL METHODS */

  //Moves the Robot:
  public void moveRobot() {
    /* Powers */

    //Cubes the Gamepad Values:
    double leftPower = (gamepad1.left_stick_y * gamepad1.left_stick_y * gamepad1.left_stick_y);
    double rightPower = (gamepad1.right_stick_y * gamepad1.right_stick_y * gamepad1.right_stick_y);
    double leftStrafe = (gamepad1.left_stick_x * gamepad1.left_stick_x * gamepad1.left_stick_x);
    double rightStrafe = (gamepad1.right_stick_x * gamepad1.right_stick_x * gamepad1.right_stick_x);

    /* Slow Mode */

    //Slow Mode Boolean:
    boolean slowMode = false;
    double divisor = 3.0;

    //Setting Slow Mode:
    if (gamepad1.left_bumper) {
      //Sets the Slow Mode:
      slowMode = true;
    }

    //Checks the Slow Mode Boolean Powers:
    if (slowMode == true) {
      //Corrects the Speed:
      leftPower /= divisor;
      rightPower /= divisor;
      leftStrafe /= divisor;
      rightStrafe /= divisor;
    }

    /* Robot Movement */

    //Forward and Backward:
    if (gamepad1.left_stick_y != 0.0 || gamepad1.right_stick_y != 0.0 && gamepad1.left_stick_x == 0.0 && gamepad1.right_stick_x == 0.0) {
      //Sets the power to the motors:
      robot.leftFrontMotor.setPower(leftPower);
      robot.leftBackMotor.setPower(leftPower);
      robot.rightFrontMotor.setPower(rightPower);
      robot.rightBackMotor.setPower(rightPower);
    }

    //Left and Right:
    else if (gamepad1.left_stick_x != 0.0 || gamepad1.right_stick_x != 0.0 && gamepad1.left_stick_y == 0.0 && gamepad1.right_stick_y == 0.0) {
      //Sets the power to the motors:
      robot.leftFrontMotor.setPower(-leftStrafe);
      robot.leftBackMotor.setPower(leftStrafe);
      robot.rightFrontMotor.setPower(rightStrafe);
      robot.rightBackMotor.setPower(-rightStrafe);
    }

    //Stop:
    else if (gamepad1.left_stick_x == 0.0 && gamepad1.right_stick_x == 0.0 && gamepad1.left_stick_y == 0.0 && gamepad1.right_stick_y == 0.0) {
      //Sets the power to the motors:
      robot.leftFrontMotor.setPower(robot.zeroPower);
      robot.leftBackMotor.setPower(robot.zeroPower);
      robot.rightFrontMotor.setPower(robot.zeroPower);
      robot.rightBackMotor.setPower(robot.zeroPower);
    }
  }

  //Moves the Arm:
  public void moveArm() {
    /* Powers */

    //Powers for the Arm:
    double armPower = 0.4;
    double armPowerUp = 0.5;

    /* Slow Mode */

    //Slow Mode Boolean:
    boolean slowMode = false;

    //Setting Slow Mode:
    if (gamepad2.left_bumper) {
      //Sets the Slow Mode
      slowMode = true;
    }

    //Checks the Slow Mode Boolean Powers:
    if (slowMode == true) {
      //Sets the Powers:
      armPower = 0.2;
      armPowerUp = 0.25;
    }

    /* Arm Movement */

    //Code for moving forearm forward:
    if (gamepad2.left_stick_y > 0.0) {
      robot.baseArmMotor.setPower(armPowerUp);
    }

    //Code for moving forearm backward:
    else if (gamepad2.left_stick_y < 0.0) {
      robot.baseArmMotor.setPower(-armPower);
    }

    //Code for stopping:
    else if (gamepad2.left_stick_y == 0.0) {
      robot.baseArmMotor.setPower(robot.zeroPower);
    }
  }

  //Grabs the Wobble Goal:
  public void grabWobble() {
    //Setting Values:
    if (gamepad2.y) {
      //Sets the Claw:
      clawClosed *= -1;

      //Opening and Closing Claws:
      if (clawClosed == -1) {
        //Sets Positions:
        robot.holdServo.setPosition(robot.holdServoStartPosition);
        robot.clawServo.setPosition(robot.clawServoStartPosition);
      }

      if (clawClosed == 1) {
        //Sets Positions:
        robot.holdServo.setPosition(robot.holdServoEndPosition);
        robot.clawServo.setPosition(robot.clawServoEndPosition);
      }
    }
  }
}
