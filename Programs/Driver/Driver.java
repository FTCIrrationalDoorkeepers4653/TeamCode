package org.firstinspires.ftc.teamcode.Programs.Driver;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Systems.Core.GamePad;
import org.firstinspires.ftc.teamcode.Systems.Core.Robot;

@TeleOp(name="Driver")
public class Driver extends LinearOpMode {
  /* DRIVE VARIABLES */

  //Objects:
  private static Robot robot = new Robot();
  private ElapsedTime time = new ElapsedTime();
  private GamePad driverPad;
  private GamePad operatorPad;

  /* RUN METHODS */

  //RunOpMode Method:
  @Override
  public void runOpMode() {
    /* Initialization */

    //Sets Status:
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    //Initialize Robot:
    robot.init(hardwareMap, false, false);
    robot.mechanisms.initMechanisms(hardwareMap);
    driverPad = new GamePad(gamepad1);
    operatorPad = new GamePad(gamepad2);

    //Wait for Start:
    waitForStart();

    /* Run */

    //Starts the Timer:
    time.reset();

    //Loop Until Stop:
    mainLoop: while (opModeIsActive()) {
      //GamePad Method Calls:
      driverPad.setGamePad();
      operatorPad.setGamePad();

      //Mechanisms MethodsL:
      moveShooter();
      moveIntake();
      moveWobble();

      //Drive Train Methods:
      moveRobot();
      aimRobot();

      //Sets Status:
      telemetry.addData("Status", "Running");
      telemetry.update();
    }

    /* Stop */

    //Sets Status:
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
      robot.mechanisms.applyControlMotorPower(robot.leftFrontMotor, -forward);
      robot.mechanisms.applyControlMotorPower(robot.leftBackMotor, -forward);
      robot.mechanisms.applyControlMotorPower(robot.rightFrontMotor, -forward);
      robot.mechanisms.applyControlMotorPower(robot.rightBackMotor, -forward);
    }

    else if (turn != robot.zeroPower) {
      //Sets the Motor Power:
      robot.mechanisms.applyControlMotorPower(robot.leftFrontMotor, turn);
      robot.mechanisms.applyControlMotorPower(robot.leftBackMotor, turn);
      robot.mechanisms.applyControlMotorPower(robot.rightFrontMotor, -turn);
      robot.mechanisms.applyControlMotorPower(robot.rightBackMotor, -turn);
    }

    else if (left != robot.zeroPower) {
      //Sets the Motor Power:
      robot.mechanisms.applyControlMotorPower(robot.leftFrontMotor, -left);
      robot.mechanisms.applyControlMotorPower(robot.leftBackMotor, left);
      robot.mechanisms.applyControlMotorPower(robot.rightFrontMotor, left);
      robot.mechanisms.applyControlMotorPower(robot.rightBackMotor, -left);
    }

    else if (right != robot.zeroPower) {
      //Sets the Motor Power:
      robot.mechanisms.applyControlMotorPower(robot.leftFrontMotor, right);
      robot.mechanisms.applyControlMotorPower(robot.leftBackMotor, -right);
      robot.mechanisms.applyControlMotorPower(robot.rightFrontMotor, -right);
      robot.mechanisms.applyControlMotorPower(robot.rightBackMotor, right);
    }

    else {
      //Sets the Motor Power:
      robot.mechanisms.applyControlMotorPower(robot.leftFrontMotor, robot.zeroPower);
      robot.mechanisms.applyControlMotorPower(robot.leftBackMotor, robot.zeroPower);
      robot.mechanisms.applyControlMotorPower(robot.rightFrontMotor, robot.zeroPower);
      robot.mechanisms.applyControlMotorPower(robot.rightBackMotor, robot.zeroPower);
    }
  }

  //Aims the Robot to Goals:
  public void aimRobot() {
    //Checks the Case:
    if (driverPad.isAReleased()) {
      //Sets Modes, Turns, and Resets:
      robot.applyAllModes(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.applyAllModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      robot.mechanisms.turnGyro(-70.0, robot.fastPower, false);
      robot.applyAllModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      robot.applyAllModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
  }
}