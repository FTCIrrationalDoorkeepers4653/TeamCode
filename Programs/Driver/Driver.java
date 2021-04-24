package org.firstinspires.ftc.teamcode.Programs.Driver;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.Core.GamePad;
import org.firstinspires.ftc.teamcode.Systems.Core.Robot;

@TeleOp(name="Driver")
public class Driver extends LinearOpMode {
  /* DRIVE VARIABLES */

  //Drive Variables:
  private Robot robot = new Robot();
  private GamePad driverPad;
  private boolean camera = false;
  private double RPM = 3420.0;

  /* OPMODE METHODS */

  @Override
  public void runOpMode() {
    //Initialize Robot:
    robot.init(hardwareMap, camera);
    robot.mechanisms.initMechanisms(hardwareMap);
    robot.mechanisms.arm = 0;
    driverPad = new GamePad(gamepad1);

    //Startup Mechanisms:
    robot.mechanisms.automateFlywheel(robot.mechanisms.mainRPM);
    robot.mechanisms.automateIntake();

    //Waits for Start:
    waitForStart();

    //Loops:
    mainLoop: while (opModeIsActive()) {
      //Move Methods:
      moveRobot();
      moveIntake();
      moveShooter();
      moveWobble();
      movePowerShots();
    }
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
    robot.driveRobot(x, y, turn, driverPad.leftBumper);
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
      robot.mechanisms.automateFlywheel(robot.mechanisms.mainRPM);
    }

    //Checks the Case:
    if (driverPad.isRightBumperReleased()) {
      //Shoots Rings:
      robot.mechanisms.automateShooter(false);
      robot.mechanisms.automateShooter(true);
      robot.mechanisms.automateShooter(true);
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
      robot.mechanisms.automateClaw();
    }
  }

  //Moves to Hit PowerShots:
  public void movePowerShots() {
    //Sets the GamePad:
    driverPad.setGamePad();

    //Checks the Case:
    if (driverPad.isDpadUpReleased()) {
      //Revs to Proper Speed:
      robot.mechanisms.shooter = 1;
      robot.mechanisms.automateFlywheel(RPM);

      //Moves and Hits First Shot:
      robot.mechanisms.runToPosition(0.0, 95.0, 1, robot.gyroPower, false);
      robot.mechanisms.automateShooter(false);

      //Moves and Hits Second Shot:
      robot.mechanisms.runToPosition(0.0, 135.0, 1, robot.gyroPower, false);
      robot.mechanisms.automateShooter(false);

      //Moves and Hits Third Shot:
      robot.mechanisms.runToPosition(0.0, 175.0, 1, robot.gyroPower, false);
      robot.mechanisms.automateShooter(false);
    }
  }
}