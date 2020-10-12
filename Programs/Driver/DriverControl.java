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

    //Wait for Start:
    waitForStart();

    /* Run */

    //Loop Until Stop:
    while (opModeIsActive()) {
      //Method Calls:
      moveRobot();
      moveArm();
      grabWobble();

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
    /* Power Controls */

    //Gets the Gamepad Control Values:
    double leftPower = (gamepad1.left_stick_y * gamepad1.left_stick_y);
    double rightPower = (gamepad1.right_stick_y * gamepad1.right_stick_y);
    double leftStrafe = (gamepad1.left_stick_x * gamepad1.left_stick_x);
    double rightStrafe = (gamepad1.right_stick_x * gamepad1.right_stick_x);

    //Gets the Step Control Values:
    double leftControl = robot.getStepFunction(leftPower);
    double rightControl = robot.getStepFunction(rightPower);
    double leftShift = robot.getStepFunction(leftStrafe);
    double rightShift = robot.getStepFunction(rightStrafe);

    //Sets the New Powers:
    leftPower *= leftControl;
    rightPower *= rightControl;
    leftStrafe *= leftShift;
    rightStrafe *= rightShift;

    /* Slow Mode */

    //Checks Slow Mode:
    if (gamepad1.right_trigger > 0.0) {
      //Sets the Slow Mode:
      leftPower /= robot.speedControl;
      rightPower /= robot.speedControl;
      leftStrafe /= robot.speedControl;
      rightStrafe /= robot.speedControl;
    }

    /* Robot Movement */

    //Checks the Case:
    if (gamepad1.left_stick_y != robot.zeroPower || gamepad1.right_stick_y != robot.zeroPower) {
      //Sets the power to the motors:
      robot.leftFrontMotor.setPower(-leftPower);
      robot.leftBackMotor.setPower(-leftPower);
      robot.rightFrontMotor.setPower(-rightPower);
      robot.rightBackMotor.setPower(-rightPower);
    }

    else if (gamepad1.left_stick_x != robot.zeroPower || gamepad1.right_stick_x != robot.zeroPower) {
      //Sets the power to the motors:
      robot.leftFrontMotor.setPower(leftStrafe);
      robot.leftBackMotor.setPower(-leftStrafe);
      robot.rightFrontMotor.setPower(-rightStrafe);
      robot.rightBackMotor.setPower(rightStrafe);
    }
  }

  //Moves the Arm:
  public void moveArm() {
    //Checks the Case:
    if (gamepad1.left_trigger > robot.zeroPower) {
      //Moves Arm Up:
      robot.baseArmMotor.setPower(robot.slowPower);
    }

    else if (gamepad1.right_trigger > robot.zeroPower) {
      //Moves Arm Down:
      robot.baseArmMotor.setPower(-robot.slowPower);
    }
  }

  //Grabs the Wobble Goal:
  public void grabWobble() {
    //Checks the Case:
    if (gamepad2.y) {
      //Checks the Case:
      if (open) {
        //Set the Values:
        robot.operateClaw("close");
        open = false;
      }

      else {
        //Sets the Values:
        robot.operateClaw("open");
        open = true;
      }
    }
  }
}
