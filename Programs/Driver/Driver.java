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

  /* OPMODE METHODS */

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

  @Override
  public void loop() {
    //Move Methods:
    moveRobot();
    moveShooter();
    moveIntake();
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
    double rightY = -robot.getSpeedControl(driverPad.rightY, driverPad.leftBumper);

    //Checks the Case:
    if (leftY > 0 && rightY == 0) {
      //Sets the Motor Powers:
      robot.leftFrontMotor.setPower(-leftY);
      robot.leftBackMotor.setPower(leftY);
      robot.rightFrontMotor.setPower(leftY);
      robot.rightBackMotor.setPower(-leftY);
    }

    else if (leftY == 0 && rightY > 0) {
      //Sets the Motor Powers:
      robot.leftFrontMotor.setPower(rightY);
      robot.leftBackMotor.setPower(-rightY);
      robot.rightFrontMotor.setPower(-rightY);
      robot.rightBackMotor.setPower(rightY);
    }

    else {
      //Sets the Motor Powers:
      robot.leftFrontMotor.setPower(leftY);
      robot.leftBackMotor.setPower(leftY);
      robot.rightFrontMotor.setPower(rightY);
      robot.rightBackMotor.setPower(rightY);
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
      robot.mechanisms.automateIntake(robot.uncoPower);
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
      robot.mechanisms.automateShooter();
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
      robot.mechanisms.automateArm(robot.mainPower);
    }

    //Checks the Case:
    if (operatorPad.isYReleased()) {
      //Operates the Claw:
      robot.mechanisms.automateClaw();
    }
  }
}