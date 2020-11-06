package org.firstinspires.ftc.teamcode.Programs.Driver;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Systems.Core.GamePad;
import org.firstinspires.ftc.teamcode.Systems.Core.Robot;

import dalvik.system.DelegateLastClassLoader;

@TeleOp(name = "DriverControl")
//@Disabled
public class DriverControl extends LinearOpMode {
  /* DRIVE VARIABLES */

  //Objects:
  private static Robot robot = new Robot();
  private ElapsedTime time = new ElapsedTime();
  private GamePad driverPad;
  private GamePad operatorPad;

  //Variables:
  private static int alignment = 0;
  private static int secondsLeft = 80;

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

      //Intake Control Methods:
      moveIntakeArm();
      moveIntakeClaw();

      //Shooter Control Methods
      moveShooter();
      moveFlywheel();

      //Arm Control Methods:
      moveArm();
      moveClaw();

      //Base Control Methods:
      moveRobot();
      alignRobot();

      //Sets Status:
      telemetry.addData("Status", "Running");
      telemetry.addData("Time Remaining", (secondsLeft - time.seconds()));
      telemetry.update();
    }

    /* Stop */

    //Sets Status:
    telemetry.addData("Status", "Stopped");
    telemetry.update();
  }

  /* DRIVER INTAKE CONTROL METHODS */

  //Moves the Intake Arm:
  public void moveIntakeArm() {
    //Checks the Case:
    if (operatorPad.isXReleased()) {
      //Checks the Case:
      if (robot.mechanisms.intakeArm == 0) {
        //Sets the Intake Arm:
        robot.mechanisms.intakeArm++;
      }

      else if (robot.mechanisms.intakeArm == 1) {
        //Sets the Intake Arm:
        robot.mechanisms.intakeArm--;
      }

      //Operates the Intake Arm:
      robot.mechanisms.automateIntake(robot.fastPower);
    }
  }

  //Moves the Intake Claw:
  public void moveIntakeClaw() {
    //Checks the Case:
    if (operatorPad.isBReleased()) {
      //Checks the Case:
      if (robot.mechanisms.intakeClaw == 0) {
        //Sets the Intake Claw:
        robot.mechanisms.intakeClaw++;
      }

      else if (robot.mechanisms.intakeClaw == 1) {
        //Sets the Intake Claw:
        robot.mechanisms.intakeClaw++;
      }

      //Operates the Intake Claw:
      robot.mechanisms.operateIntakeClaw();
    }
  }

  /* DRIVER SHOOTER CONTROL METHODS */

  //Moves the Shooter:
  public void moveShooter() {
    //Checks the Case:
    if (operatorPad.isRightBumperReleased()) {
      //Checks the Case:
      if (robot.mechanisms.shooter == 0 && robot.mechanisms.ringCount > 0) {
        //Turns On Flywheel:
        robot.mechanisms.automateFlywheel();
        robot.mechanisms.completeCycle(robot.mechanisms.shooterWait);
      }

      //Shoots the Ring:
      robot.mechanisms.automateShooter();
    }
  }

  //Moves the Flywheel:
  public void moveFlywheel() {
    //Checks the Case:
    if (operatorPad.isLeftBumperReleased()) {
      //Checks the Case:
      if (robot.mechanisms.shooter == 0) {
        //Sets the Shooter:
        robot.mechanisms.shooter++;
      }

      else if (robot.mechanisms.shooter == 1) {
        //Sets the Shooter:
        robot.mechanisms.shooter--;
      }
    }

    //Operates the Flywheel:
    robot.mechanisms.operateFlywheel();
  }

  /* DRIVER ARM CONTROL METHODS */

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
      robot.mechanisms.automateArm(robot.fastPower);
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

  /* DRIVER BASE CONTROL METHODS */

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

  //Aligns Robot to PowerShots:
  public void alignRobot() {
    //Checks the Case:
    if (driverPad.isAReleased()) {
      //Sets Modes, Turns, and Resets:
      robot.applyAllModes(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.mechanisms.turnGyro("right", 90.0, robot.fastPower);
      robot.applyAllModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Checks the Case:
    if (driverPad.isYReleased()) {
      //Checks the Case:
      if (alignment == 0) {
        //Sets Modes, Turns, and Resets:
        robot.applyAllModes(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.mechanisms.turnGyro("right", 60.0, robot.fastPower);
        robot.applyAllModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        alignment++;
      }

      else if (alignment == 1) {
        //Sets Modes, Turns, and Resets:
        robot.applyAllModes(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.mechanisms.turnGyro("right", 15.0, robot.fastPower);
        robot.applyAllModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        alignment++;
      }

      else if (alignment == 2) {
        //Sets Modes, Turns, and Resets:
        robot.applyAllModes(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.mechanisms.turnGyro("left", 10.0, robot.fastPower);
        robot.applyAllModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        alignment -= 2;
      }
    }
  }
}