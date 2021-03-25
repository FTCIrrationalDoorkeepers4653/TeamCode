package org.firstinspires.ftc.teamcode.Programs.Driver;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Systems.Core.GamePad;
import org.firstinspires.ftc.teamcode.Systems.Core.Robot;

@TeleOp(name="Driver")
public class Driver extends OpMode {
  /* DRIVE VARIABLES */

  //Objects:
  private Robot robot = new Robot();
  private GamePad driverPad;

  //Setup Variables:
  private int values[] = {0, 2, 0, 0, 0};
  private boolean auto = false;

  /* OPMODE METHODS */

  @Override
  public void init() {
    //Initialize Robot:
    robot.init(hardwareMap, false, false);
    robot.mechanisms.initMechanisms(hardwareMap, auto);
    robot.mechanisms.initCustomValues(values);
    driverPad = new GamePad(gamepad1);
  }

  @Override
  public void loop() {
    //Move Methods:
    moveRobot();
    moveIntake();
    moveShooter();
    moveWobble();
    moveAuto();
  }

  /* MOVEMENT METHODS */

  //Moves the Robot:
  public void moveRobot() {
    //Sets the GamePad Values:
    driverPad.setGamePad();

    //Gets the Movement Variables:
    double x = robot.getStrafeValue(driverPad.leftTrigger, driverPad.rightTrigger);
    double y = -driverPad.leftY;
    double turn = driverPad.rightX;

    //Sets Motor Powers:
    robot.driveRobot(x, y, turn);
  }

  //Moves the Intake Arm:
  public void moveIntake() {
    //Sets the GamePad:
    driverPad.setGamePad();

    //Checks the Case:
    if (driverPad.isXReleased()) {
      //Operates the Intake Arm:
      robot.mechanisms.automateIntake();
    }
  }

  //Moves the Shooter:
  public void moveShooter() {
    //Sets the GamePad:
    driverPad.setGamePad();

    //Checks the Case:
    if (driverPad.isBReleased()) {
      //Operates the Flywheel:
      robot.mechanisms.automateFlywheel(true);
    }

    //Checks the Case:
    if (driverPad.isRightBumperReleased()) {
      //Shoots Rings:
      robot.mechanisms.automateShooter(0);
      robot.mechanisms.automateShooter(robot.mechanisms.shooterWait);
      robot.mechanisms.automateShooter(robot.mechanisms.shooterWait);
    }
  }

  //Moves the Wobble Claw:
  public void moveWobble() {
    //Sets the GamePad:
    driverPad.setGamePad();

    //Checks the Case:
    if (driverPad.isAReleased()) {
      //Moves the Arm:
      robot.mechanisms.automateArm();
    }

    //Checks the Case:
    if (driverPad.isYReleased()) {
      //Operates the Claw:
      robot.mechanisms.automateClaw(true);
    }
  }

  //Movement Automation:
  public void moveAuto() {
    //Sets the GamePad Values:
    driverPad.setGamePad();

    //Checks the Case:
    if (driverPad.isLeftBumperReleased()) {
      //Initial Startup:
      robot.mechanisms.intake = 1;
      robot.mechanisms.shooter = 0;
      robot.mechanisms.automateIntake();
      robot.mechanisms.automateFlywheel(true);

      //Robot Moves Forward and Shoots:
      robot.mechanisms.runToPosition(0.0, 12.0, 1, robot.firePower, false);
      robot.mechanisms.automateShooter(0);

      //Resets Shooter RPM:
      robot.mechanisms.shooter = 0;
      robot.mechanisms.automateFlywheel(false);

      //Robot Turns Left and Shoots:
      robot.mechanisms.turnGyro(4.0, robot.firePower, false);
      robot.mechanisms.automateShooter(robot.mechanisms.shooterWait);

      //Robot Turns Left and Shoots:
      robot.mechanisms.turnGyro(4.0, robot.firePower, false);
      robot.mechanisms.automateShooter(robot.mechanisms.shooterWait);
      robot.mechanisms.resetCurrentPosition();
    }
  }
}