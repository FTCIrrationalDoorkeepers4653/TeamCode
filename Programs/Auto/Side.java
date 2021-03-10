package org.firstinspires.ftc.teamcode.Programs.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Systems.Core.Robot;

@Autonomous(name="Side")
@Disabled
public class Side extends LinearOpMode {
  /* SIDE AUTO VARIABLES */

  //Movement Variables:
  private Robot robot = new Robot();
  private double startX = 560.0;
  private double startY = 760.0;

  //Positioning Variables:
  private int position = 0;
  private int values[] = {1, 0, 0, 0, 0, 0};

  //Setup Variables:
  private boolean auto = true;
  private boolean teleOp = false;

  /* OPMODE METHODS */

  @Override
  public void runOpMode() {
    /* Initialization */

    //Status Updates:
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    //Hardware Initialization:
    robot.init(hardwareMap, true, true);
    robot.mechanisms.initMechanisms(hardwareMap, auto);
    robot.mechanisms.initCustomValues(values);
    robot.mechanisms.setCurrentPosition(startX, startY, robot.getTheta());

    //Waits for Start:
    waitForStart();

    /* Detection */

    //Gets the Ring Position and Setup:
    robot.mechanisms.automateFlywheel(auto);
    position = robot.getPixelsPosition();

    //Moves to Shooting Position:
    robot.mechanisms.runToPosition(560.0, 410.0, 1, robot.firePower, true);
    robot.mechanisms.turnGyro(-85.0, robot.firePower, true);

    /* Shooting */

    //Shoots Rings:
    robot.mechanisms.automateShooter(0);
    robot.mechanisms.automateShooter(robot.mechanisms.shooterWait);
    robot.mechanisms.automateShooter(robot.mechanisms.shooterWait);

    //Resets:
    robot.mechanisms.automateFlywheel(auto);
    robot.mechanisms.automatePark();

    /* Wobble Goal Drop */

    //Checks the Case:
    if (position == 1) {
      //Moves Wobble:
      robot.mechanisms.turnGyro(5.0, robot.firePower, true);
      robot.mechanisms.runToPosition(660.0, 380.0, 1, robot.firePower, true);

      //Drops Wobble and Parks:
      robot.mechanisms.automateClaw(teleOp);
      robot.mechanisms.automateArm();
      robot.mechanisms.automateArm();
    }

    else if (position == 2) {
      //Moves Wobble:
      robot.mechanisms.turnGyro(55.0, robot.firePower, true);
      robot.mechanisms.runToPosition(600.0, 360.0, 1, robot.firePower, true);

      //Drops Wobble and Parks:
      robot.mechanisms.automateClaw(teleOp);
      robot.mechanisms.automateArm();
      robot.mechanisms.automateArm();
    }

    else {
      //Moves Wobble:
      robot.mechanisms.turnGyro(52.0, robot.firePower, true);
      robot.mechanisms.runToPosition(640.0, 160.0, 1, robot.firePower, true);

      //Drops Wobble and Parks:
      robot.mechanisms.automateClaw(teleOp);
      robot.mechanisms.automateArm();
      robot.mechanisms.runToPosition(640.0, 340.0, -1, robot.firePower, true);
      robot.mechanisms.automateArm();
    }

    /* Stop */

    //Status Update:
    telemetry.addData("Status", "Stopped");
    telemetry.update();
  }
}