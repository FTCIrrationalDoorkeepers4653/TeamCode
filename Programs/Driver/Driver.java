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
  private int values[] = {0, 2, 0, 0, 0, 0};

  /* OPMODE METHODS */

  @Override
  public void init() {
    //Status Updates:
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    //Initialize Robot:
    robot.init(hardwareMap, false, false);
    robot.mechanisms.initMechanisms(hardwareMap);
    robot.mechanisms.initCustomValues(values);
    driverPad = new GamePad(gamepad1);
    operatorPad = new GamePad(gamepad2);
  }

  @Override
  public void loop() {
    //Move Methods:
    moveRobot();
    moveIntake();
    moveShooter();
    moveWobble();
  }

  @Override
  public void stop() {
    //Status Updates:
    telemetry.addData("Status", "Stopped");
    telemetry.update();
  }

  /* MOVEMENT METHODS */

  //Moves the Robot:
  public void moveRobot() {
    //Sets the GamePads:
    driverPad.setGamePad();
    operatorPad.setGamePad();

    //Gets the GamePad Control Values:
    double leftY = -robot.getSpeedControl(driverPad.leftY, driverPad.leftBumper);
    double rightX = -robot.getSpeedControl(driverPad.rightX, driverPad.leftBumper);
    double leftTrigger = -robot.getSpeedControl(driverPad.leftTrigger, driverPad.leftBumper);
    double rightTrigger = -robot.getSpeedControl(driverPad.rightTrigger, driverPad.leftBumper);

    //Checks the Case:
    if (leftY != 0) {
      //Sets the Motor Powers:
      robot.leftFrontMotor.setPower(leftY);
      robot.leftBackMotor.setPower(leftY);
      robot.rightFrontMotor.setPower(leftY);
      robot.rightBackMotor.setPower(leftY);
    }

    else if (rightX != 0) {
      //Sets the Motor Powers:
      robot.leftFrontMotor.setPower(-rightX);
      robot.leftBackMotor.setPower(-rightX);
      robot.rightFrontMotor.setPower(rightX);
      robot.rightBackMotor.setPower(rightX);
    }

    else if (leftTrigger != 0) {
      //Sets the Motor Powers:
      robot.leftFrontMotor.setPower(leftTrigger);
      robot.leftBackMotor.setPower(-leftTrigger);
      robot.rightFrontMotor.setPower(-leftTrigger);
      robot.rightBackMotor.setPower(leftTrigger);
    }

    else if (rightTrigger != 0) {
      //Sets the Motor Powers:
      robot.leftFrontMotor.setPower(-rightTrigger);
      robot.leftBackMotor.setPower(rightTrigger);
      robot.rightFrontMotor.setPower(rightTrigger);
      robot.rightBackMotor.setPower(-rightTrigger);
    }

    else {
      //Sets the Motor Powers:
      robot.leftFrontMotor.setPower(robot.zeroPower);
      robot.leftBackMotor.setPower(robot.zeroPower);
      robot.rightFrontMotor.setPower(robot.zeroPower);
      robot.rightBackMotor.setPower(robot.zeroPower);
    }
  }

  //Moves the Intake Arm:
  public void moveIntake() {
    //Sets the GamePads:
    driverPad.setGamePad();
    operatorPad.setGamePad();

    //Checks the Case:
    if (operatorPad.isXReleased()) {
      //Operates the Intake Arm:
      robot.mechanisms.automateIntake();
    }

    //Checks the Case:
    if (operatorPad.isBReleased()) {
      //Operates the Intake Claw:
      robot.mechanisms.automateIntakeClaw();
    }
  }

  //Moves the Shooter:
  public void moveShooter() {
    //Sets the GamePads:
    driverPad.setGamePad();
    operatorPad.setGamePad();

    //Checks the Case:
    if (operatorPad.isLeftBumperReleased()) {
      //Operates the Flywheel:
      robot.mechanisms.automateFlywheel();
    }

    //Checks the Case:
    if (operatorPad.isRightBumperReleased()) {
      //Shoots the Ring:
      robot.mechanisms.automateShooter(true, true);
    }
  }

  //Moves the Wobble Claw:
  public void moveWobble() {
    //Sets the GamePads:
    driverPad.setGamePad();
    operatorPad.setGamePad();

    //Checks the Case:
    if (operatorPad.isAReleased()) {
      //Moves the Arm:
      robot.mechanisms.automateArm();
    }

    //Checks the Case:
    if (operatorPad.isYReleased()) {
      //Operates the Claw:
      robot.mechanisms.automateClawTele();
    }
  }
}