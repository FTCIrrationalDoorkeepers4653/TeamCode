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
  double clawClosed = -1;

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

    //Checks the Case:
    if (gamepad1.left_stick_y != 0.0 || gamepad1.right_stick_y != 0.0 && gamepad1.left_stick_x == 0.0 && gamepad1.right_stick_x == 0.0) {
      //Sets the power to the motors:
      robot.leftFrontMotor.setPower(-leftPower);
      robot.leftBackMotor.setPower(-leftPower);
      robot.rightFrontMotor.setPower(-rightPower);
      robot.rightBackMotor.setPower(-rightPower);
    }

    //Checks the Case:
    if (gamepad1.left_stick_x != 0.0 || gamepad1.right_stick_x != 0.0 && gamepad1.left_stick_y == 0.0 && gamepad1.right_stick_y == 0.0) {
      //Sets the power to the motors:
      robot.leftFrontMotor.setPower(leftStrafe);
      robot.leftBackMotor.setPower(-leftStrafe);
      robot.rightFrontMotor.setPower(-rightStrafe);
      robot.rightBackMotor.setPower(rightStrafe);
    }
  }

  //Moves the Arm:
  public void moveArm() {
    /* Arm Movement */

    //Checks the Case:
    if (gamepad1.left_trigger > 0.0) {
      //Moves Arm Up:
      robot.baseArmMotor.setPower(robot.slowPower);
    }

    else if (gamepad1.right_trigger > 0.0) {
      //Moves Arm Down:
      robot.baseArmMotor.setPower(-robot.slowPower);
    }
  }

  //Grabs the Wobble Goal:
  public void grabWobble() {
    /* Claw Movement */

    //Setting Values:
    if (gamepad2.y) {
      //Sets the Claw:
      clawClosed *= -1;

      //Opening and Closing Claws:
      if (clawClosed == -1) {
        //Sets Positions:
        robot.operateClaw("open");
      }

      else if (clawClosed == 1) {
        //Sets Positions:
        robot.operateClaw("close");
      }
    }
  }
}
