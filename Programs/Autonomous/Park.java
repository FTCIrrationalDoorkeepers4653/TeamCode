package org.firstinspires.ftc.teamcode.Programs.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Systems.Core.Robot;

@Autonomous(name = "Park")
//@Disabled
public class Park extends LinearOpMode {
  /* PARK VARIABLES */

  //Robot Objects:
  private Robot robot = new Robot();

  //Movement Variables:
  private double startCoordinates[] = new double[2];
  private double endCoordinates[] = new double[2];

  /* OPMODE METHODS */

  @Override
  public void runOpMode() {
    /* Initialization */

    //Status Updates:
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    //Hardware Initialization:
    robot.init(hardwareMap, true, false);
    robot.mechanisms.initMechanisms();

    //Waits for Start:
    waitForStart();

    /* Wobble Goal and Park */

    //Moves to Drop Wobble:
    startCoordinates[0] = 652.0;
    startCoordinates[1] = 760.0;
    endCoordinates[0] = 652.0;
    endCoordinates[1] = 125.0;
    robot.mechanisms.runToPosition("forward", 1, startCoordinates, endCoordinates, robot.mainPower);

    //Drops Wobble:
    robot.mechanisms.arm = 1;
    robot.mechanisms.automateArm(robot.mainPower);

    startCoordinates[0] = endCoordinates[0];
    startCoordinates[1] = endCoordinates[1];
    endCoordinates[0] = 652.0;
    endCoordinates[1] = 360.0;
    robot.mechanisms.runToPosition("backward", 1, startCoordinates, endCoordinates, robot.mainPower);

    //Resets the Arm:
    robot.mechanisms.arm = 0;
    robot.mechanisms.automateArm(robot.mainPower);

    /* Stop */

    //Status Update:
    telemetry.addData("Status", "Stopped");
    telemetry.update();
  }
}
