package org.firstinspires.ftc.teamcode.Programs.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Systems.Movement.IDKRobot;

@TeleOp(name = "IDKTeleOp")
public class IDKTeleOp extends OpMode {
  //Runtime Object:
  private ElapsedTime runtime = new ElapsedTime();
  IDKRobot robot = new IDKRobot();

  //Motor Variables:
  public static final double zeroPower = 0.0;
  boolean runType = false;
  double clawClosed = -1;

  //Runs code once upon INIT:
  @Override
  public void init() {
    //Init Robot:
    robot.init(hardwareMap, runType);

    //Sets Status:
    telemetry.addData("Status: ", "Initalized");
    telemetry.update();
  }

  //Runs code repeatedly upon START:
  @Override
  public void loop() {
    //Movement Method Calls:
    moveRobot();
    moveArm();
    grabWobble();

    //Sets Status:
    telemetry.addData("Status: ", "Running");
    telemetry.update();
  }

  /* Motor Methods: */

  //Code responsible for the movement of the whole robot:
  public void moveRobot() {
    //Cubes the gamepad values so motors do not get stressed:
    double leftPower = (gamepad1.left_stick_y * gamepad1.left_stick_y * gamepad1.left_stick_y);
    double rightPower = (gamepad1.right_stick_y * gamepad1.right_stick_y * gamepad1.right_stick_y);

    //Strafe values (also cubed):
    double leftStrafe = (gamepad1.left_stick_x * gamepad1.left_stick_x * gamepad1.left_stick_x);
    double rightStrafe = (gamepad1.right_stick_x * gamepad1.right_stick_x * gamepad1.right_stick_x);

    //Slow Mode Boolean:
    boolean slowMode = false;

    //Setting Slow Mode:
    if (gamepad1.left_bumper) {
      slowMode = true;
    }

    //Checks the Slow Mode Boolean Powers:
    if (slowMode == true) {
      leftPower /= 3.0;
      rightPower /= 3.0;
      leftStrafe /= 3.0;
      rightStrafe /= 3.0;
    }

    //Forward and Backward:
    if (gamepad1.left_stick_y != 0.0 || gamepad1.right_stick_y != 0.0 && gamepad1.left_stick_x == 0.0 && gamepad1.right_stick_x == 0.0) {
      //Sets the power to the motors:
      robot.leftFrontMotor.setPower(leftPower);
      robot.leftBackMotor.setPower(leftPower);
      robot.rightFrontMotor.setPower(rightPower);
      robot.rightBackMotor.setPower(rightPower);
    }

    //Left and Right:
    if (gamepad1.left_stick_x != 0.0 || gamepad1.right_stick_x != 0.0 && gamepad1.left_stick_y == 0.0 && gamepad1.right_stick_y == 0.0) {
      //Sets the power to the motors:
      robot.leftFrontMotor.setPower(-leftStrafe);
      robot.leftBackMotor.setPower(leftStrafe);
      robot.rightFrontMotor.setPower(rightStrafe);
      robot.rightBackMotor.setPower(-rightStrafe);
    }

    //Diagonal:
    if (gamepad1.left_stick_y > 0.0 && gamepad1.left_stick_x > 0.0 || gamepad1.right_stick_y > 0.0 && gamepad1.right_stick_x > 0.0 ||
        gamepad1.left_stick_y < 0.0 && gamepad1.left_stick_x < 0.0 || gamepad1.right_stick_y < 0.0 && gamepad1.right_stick_x < 0.0) {
      robot.leftFrontMotor.setPower(leftPower);
      robot.rightBackMotor.setPower(rightPower);
    }

    if (gamepad1.left_stick_y < 0.0 && gamepad1.left_stick_x > 0.0 || gamepad1.right_stick_y < 0.0 && gamepad1.right_stick_x > 0.0 ||
        gamepad1.left_stick_y > 0.0 && gamepad1.left_stick_x < 0.0 || gamepad1.right_stick_y > 0.0 && gamepad1.right_stick_x < 0.0) {
      robot.leftBackMotor.setPower(leftPower);
      robot.rightFrontMotor.setPower(rightPower);
    }

    //Stop:
    if (gamepad1.left_stick_x == 0.0 && gamepad1.right_stick_x == 0.0 && gamepad1.left_stick_y == 0.0 && gamepad1.right_stick_y == 0.0) {
      //Sets the power to the motors:
      robot.leftFrontMotor.setPower(zeroPower);
      robot.leftBackMotor.setPower(zeroPower);
      robot.rightFrontMotor.setPower(zeroPower);
      robot.rightBackMotor.setPower(zeroPower);
    }
  }

  //Code responsible for moving the arm:
  public void moveArm() {
    //Powers for the Arm:
    double armPower;
    double armPowerUp;

    //Slow Mode Boolean:
    boolean slowMode = false;

    //Setting Slow Mode:
    if (gamepad2.left_bumper) {
      slowMode = true;
    } else if (gamepad2.right_bumper) {
      slowMode = false;
    }

    //Checks the Slow Mode Boolean Powers:
    if (slowMode == true) {
      armPower = 0.2;
      armPowerUp = 0.25;
    } else {
      armPower = 0.4;
      armPowerUp = 0.5;
    }

    //Code for moving forearm forward:
    if (gamepad2.left_stick_y > 0.0) {
      robot.baseArmMotor.setPower(armPowerUp);
    }

    //Code for moving forearm backward:
    if (gamepad2.left_stick_y < 0.0) {
      robot.baseArmMotor.setPower(-armPower);
    }

    //Code for stopping:
    if (gamepad2.left_stick_y == 0.0) {
      robot.baseArmMotor.setPower(zeroPower);
    }
  }

  /* Servo Methods */

  //Code responsible for claw servos:
  public void grabWobble() {
    //Setting Values:
    if (gamepad2.y) {
      clawClosed *= -1;

      //Opening and Closing Claws:
      if (clawClosed == -1) {
        robot.holdServo.setPosition(robot.holdServoStartPosition);
        robot.clawServo.setPosition(robot.clawServoStartPosition);
      }

      if (clawClosed == 1) {
        robot.holdServo.setPosition(robot.holdServoEndPosition);
        robot.clawServo.setPosition(robot.clawServoEndPosition);
      }
    }
  }

  //Runs code once upon STOP:
  @Override
  public void stop() {
    //Sets Status:
    telemetry.addData("Status: ", "Stopped");
    telemetry.update();
  }
}
