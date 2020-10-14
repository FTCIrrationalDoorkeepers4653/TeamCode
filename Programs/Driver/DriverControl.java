package org.firstinspires.ftc.teamcode.Programs.Driver;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Systems.Core.Robot;

@TeleOp(name = "DriverControl")
//@Disabled
public class DriverControl extends LinearOpMode {
  /* TELEOP VARIABLES */

  //DriverControl Objects and Variables:
  Robot robot = new Robot();
  boolean open = false;

  /* TELEOP METHODS */

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
    while (opModeIsActive()) {
      //Method Calls:
      moveRobot();
      moveArm();

      //Sets Status:
      telemetry.addData("Status", "Running");
      telemetry.update();
    }

    /* Stop */

    //Sets Status:
    telemetry.addData("Status", "Stopped");
    telemetry.update();
  }

  /* CONTROL METHODS */

  //Moves the Robot:
  public void moveRobot() {
    //Gets the Gamepad Control Values:
    double power = (gamepad1.left_stick_y * gamepad1.left_stick_y * gamepad1.left_stick_y);
    double turn = (gamepad1.left_stick_x * gamepad1.left_stick_x * gamepad1.left_stick_x);
    double strafe = (gamepad1.right_stick_x * gamepad1.right_stick_x * gamepad1.right_stick_x);

    //Checks the Case:
    if (power != robot.zeroPower) {
      //Sets the Motor Power:
      robot.leftFrontMotor.setPower(-power);
      robot.leftBackMotor.setPower(-power);
      robot.rightFrontMotor.setPower(-power);
      robot.rightBackMotor.setPower(-power);
    }

    else if (turn != robot.zeroPower) {
      //Sets the Motor Power:
      robot.leftFrontMotor.setPower(turn);
      robot.leftBackMotor.setPower(turn);
      robot.rightFrontMotor.setPower(-turn);
      robot.rightBackMotor.setPower(-turn);
    }

    else if (strafe != robot.zeroPower) {
      //Sets the Motor Power:
      robot.leftFrontMotor.setPower(strafe);
      robot.leftBackMotor.setPower(-strafe);
      robot.rightFrontMotor.setPower(-strafe);
      robot.rightBackMotor.setPower(strafe);
    }

    else {
      //Sets the Motor Power:
      robot.leftFrontMotor.setPower(robot.zeroPower);
      robot.leftBackMotor.setPower(robot.zeroPower);
      robot.rightFrontMotor.setPower(robot.zeroPower);
      robot.rightBackMotor.setPower(robot.zeroPower);
    }
  }

  //Grabs the Wobble Goal:
  public void moveArm() {
    //Checks the Case:
    if (gamepad1.a) {
      //Moves the Arm:
      robot.mechanisms.automateArm();
    }

    else if (gamepad1.y) {
      //Operates the Claw:
      robot.mechanisms.operateClaw();
    }

    else {
      //Resets Arm:
      robot.mechanisms.mechanismsFinishRun();
    }
  }
}