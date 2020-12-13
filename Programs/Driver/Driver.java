package org.firstinspires.ftc.teamcode.Programs.Driver;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Systems.Core.GamePad;
import org.firstinspires.ftc.teamcode.Systems.Core.Robot;

@TeleOp(name="Driver")
public class Driver extends OpMode {
  /* DRIVE VARIABLES */

  //Objects:
  private Robot robot = new Robot();
  private ElapsedTime time = new ElapsedTime();
  private GamePad driverPad;
  private GamePad operatorPad;

  /* RUN METHODS */

  //Init Method:
  @Override
  public void init() {
    //Status Updates:
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    //Initialize Robot:
    robot.init(hardwareMap, false, false);
    robot.mechanisms.initMechanisms(hardwareMap);
    driverPad = new GamePad(gamepad1);
    operatorPad = new GamePad(gamepad2);
  }

  //Loop Method:
  public void loop() {
    //GamePad Methods:
    driverPad.setGamePad();
    operatorPad.setGamePad();

    //Drive Train Methods:
    moveRobot();
    aimRobot();

    //Mechanisms Methods:
    moveShooter();
    moveIntake();
    moveWobble();
  }

  //Stop Method:
  public void stop() {
    //Status Updates:
    telemetry.addData("Status", "Stopped");
    telemetry.update();
  }

  /* MECHANISMS METHODS */

  //Moves the Intake Arm:
  public void moveIntake() {
    //Checks the Case:
    if (operatorPad.isXReleased()) {
      //Operates the Intake Arm:
      robot.mechanisms.automateIntake(robot.slowPower);
    }

    //Checks the Case:
    if (operatorPad.isBReleased()) {
      //Operates the Intake Wheel:
      robot.mechanisms.automateIntakeWheel();
    }
  }

  //Moves the Shooter:
  public void moveShooter() {
    //Checks the Case:
    if (operatorPad.isLeftBumperReleased()) {
      //Operates the Flywheel:
      robot.mechanisms.automateFlywheel();
    }

    //Checks the Case:
    if (operatorPad.isRightBumperReleased()) {
      //Shoots the Ring:
      robot.mechanisms.automateShooter();
    }
  }

  //Moves the Wobble Claw:
  public void moveWobble() {
    //Checks the Case:
    if (operatorPad.isAReleased()) {
      //Moves the Arm:
      robot.mechanisms.automateArm(robot.mainPower);
    }

    //Checks the Case:
    if (operatorPad.isYReleased()) {
      //Operates the Claw:
      robot.mechanisms.automateClaw();
    }
  }

  /* DRIVE TRAIN METHODS */

  //Moves the Robot:
  public void moveRobot() {
    //Gets the GamePad Control Values:
    double forward = (driverPad.leftY * driverPad.leftY * driverPad.leftY);
    double turn = (driverPad.rightX * driverPad.rightX * driverPad.rightX);
    double left = (driverPad.leftTrigger * driverPad.leftTrigger * driverPad.leftTrigger);
    double right = (driverPad.rightTrigger * driverPad.rightTrigger * driverPad.rightTrigger);

    //Checks the Case:
    if (driverPad.leftBumper) {
      //Adjusts Slow Mode Speed:
      forward /= robot.speedControl;
      turn /= robot.speedControl;
      left /= robot.speedControl;
      right /= robot.speedControl;
    }

    //Checks the Case:
    if (forward != robot.zeroPower) {
      //Sets the Motor Power:
      robot.leftFrontMotor.setPower(-forward);
      robot.leftBackMotor.setPower(-forward);
      robot.rightFrontMotor.setPower(-forward);
      robot.rightBackMotor.setPower(-forward);
    }

    else if (turn != robot.zeroPower) {
      //Sets the Motor Power:
      robot.leftFrontMotor.setPower(turn);
      robot.leftBackMotor.setPower(turn);
      robot.rightFrontMotor.setPower(-turn);
      robot.rightBackMotor.setPower(-turn);
    }

    else if (left != robot.zeroPower) {
      //Sets the Motor Power:
      robot.leftFrontMotor.setPower(-left);
      robot.leftBackMotor.setPower(left);
      robot.rightFrontMotor.setPower(left);
      robot.rightBackMotor.setPower(-left);
    }

    else if (right != robot.zeroPower) {
      //Sets the Motor Power:
      robot.leftFrontMotor.setPower(right);
      robot.leftBackMotor.setPower(-right);
      robot.rightFrontMotor.setPower(-right);
      robot.rightBackMotor.setPower(right);
    }

    else {
      //Sets the Motor Power:
      robot.leftFrontMotor.setPower(robot.zeroPower);
      robot.leftBackMotor.setPower(robot.zeroPower);
      robot.rightFrontMotor.setPower(robot.zeroPower);
      robot.rightBackMotor.setPower(robot.zeroPower);
    }
  }

  //Aims the Robot to Goals:
  public void aimRobot() {
    //Checks the Case:
    if (driverPad.isAReleased()) {
      //Sets Modes, Turns, and Resets:
      robot.applyAllModes(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.applyAllModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      robot.mechanisms.turnGyro(-72.0, robot.fastPower, false);
      robot.applyAllModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      robot.applyAllModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
  }
}