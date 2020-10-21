package org.firstinspires.ftc.teamcode.Programs.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Systems.Core.Robot;

@Autonomous(name = "Park")
//@Disabled
public class Park extends LinearOpMode {
  /* PARK VARIABLES */

  //Robot Objects:
  Robot robot = new Robot();

  //Movement Variables:
  double startCoordinates[] = new double[2];
  double endCoordinates[] = new double[2];

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

    /* Park */

    //Parks the Robot:
    startCoordinates[0] = 652.0;
    startCoordinates[1] = 760.0;
    endCoordinates[0] = 652.0;
    endCoordinates[1] = 380.0;
    robot.controller.runToPosition("forward", 1, startCoordinates, endCoordinates, robot.contPower);

    /* Stop */

    //Status Update:
    telemetry.addData("Status", "Stopped");
    telemetry.update();
  }
}
