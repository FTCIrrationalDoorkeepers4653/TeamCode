package org.firstinspires.ftc.teamcode.Programs.Driver;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.Core.Robot;

@TeleOp(name="FSD")
public class FSD extends OpMode {
  /* FSD VARIABLES */

  //Positioning Variables:
  private Robot robot = new Robot();
  private double turnValue = 20.0;

  //Setup Variables:
  private int values[] = {0, 2, 0, 0, 0};
  private boolean auto = true;
  private boolean camera = true;

  /* FSD RUN METHODS */

  @Override
  public void init() {
    //Initialize Robot:
    robot.init(hardwareMap, auto, camera);
    robot.mechanisms.initMechanisms(hardwareMap, auto);
    robot.mechanisms.initCustomValues(values);
  }

  @Override
  public void loop() {
    //Runs the FSD:
    FSD();
  }

  /* FSD METHODS */

  //Full Self Driving Method:
  public void FSD() {
    //Gets the Detection:
    double info[] = robot.getObjects();

    //Checks the Case:
    if (info[2] == 0) {
      //Turns Robot:
      robot.mechanisms.turnGyro(turnValue, robot.firePower, true);

      telemetry.clear();
    }

    else {
//      //Operates Mechanisms:
//      robot.mechanisms.automateIntake(robot.gyroPower);
//      robot.mechanisms.automateFlywheel(true);

      //Gets the Distances:
      double inchesToCoordinates = (info[0] / robot.POSITION_RATIO);
      double turnAngle = 0;

      telemetry.addData("Distance", inchesToCoordinates);
      telemetry.addData("Offset", info[1]);
      telemetry.addData("Blobs", info[2]);
      telemetry.update();

//      //Runs Robot to Intake:
//      robot.mechanisms.turnGyro(turnAngle, robot.gyroPower, true);
//      robot.mechanisms.runToPosition(0, inchesToCoordinates, 1, robot.gyroPower, true);
//
//      //Runs Robot Back to Shoot:
//      robot.mechanisms.runToPosition(0, 0, -1, robot.gyroPower, true);
//      robot.mechanisms.turnGyro(-robot.getTheta(), robot.gyroPower, true);
//      robot.mechanisms.automateShooter(0);
//
//      //Operates Mechanisms:
//      robot.mechanisms.automateIntake(robot.gyroPower);
//      robot.mechanisms.automateFlywheel(true);
    }
  }
}