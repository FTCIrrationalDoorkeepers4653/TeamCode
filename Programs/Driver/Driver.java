package org.firstinspires.ftc.teamcode.Programs.Driver;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.Core.GamePad;
import org.firstinspires.ftc.teamcode.Systems.Core.Robot;

@TeleOp(name="Driver")
public class Driver extends OpMode {
  /* DRIVE VARIABLES */

  //Drive Variables:
  private Robot robot = new Robot();
  private GamePad driverPad;

  //Setup Variables:
  private int values[] = {0, 2, 0, 0, 0};
  private boolean auto = false;
  private boolean camera = false;

  /* OPMODE METHODS */

  @Override
  public void init() {
    //Initialize Robot:
    robot.init(hardwareMap, auto, camera);
    robot.mechanisms.initMechanisms(hardwareMap, auto);
    robot.mechanisms.initCustomValues(values);
    driverPad = new GamePad(gamepad1);

    //Startup Mechanisms:
    robot.mechanisms.automateFlywheel(true);
    robot.mechanisms.automateIntake(robot.gyroPower);
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
      robot.mechanisms.automateIntake(robot.gyroPower);
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
    if (driverPad.isDpadUpReleased()) {
      //Robot Moves Forward and Shoots:
      robot.mechanisms.runToPosition(0.0, 10.0, 1, robot.gyroPower, false);
      robot.mechanisms.automateShooter(0);

      //Robot Turns Left and Shoots:
      robot.mechanisms.turnGyro(5.0, robot.gyroPower, false);
      robot.mechanisms.automateShooter(robot.mechanisms.shooterWait);

      //Robot Turns Left and Shoots:
      robot.mechanisms.turnGyro(5.0, robot.gyroPower, false);
      robot.mechanisms.automateShooter(robot.mechanisms.shooterWait);
      robot.mechanisms.resetCurrentPosition();
    }
  }
}