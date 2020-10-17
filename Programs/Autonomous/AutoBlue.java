package org.firstinspires.ftc.teamcode.Programs.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Systems.Core.Robot;

@Autonomous(name="AutoBlue")
//@Disabled
public class AutoBlue extends LinearOpMode {
  /* AUTO BLUE VARIABLES */

  //Robot Objects:
  Robot robot = new Robot();

  //Movement Variables:
  double startCoordinates[] = new double[2];
  double  endCoordinates[] = new double[2];
  int position = 0;

  @Override
  public void runOpMode() {
    /* Initialization */

    //Status Updates:
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    //Hardware Initialization:
    robot.init(hardwareMap, true, true);
    robot.mechanisms.initMechanisms();

    //Waits for Start:
    waitForStart();

    /* Detection */

    //Gets the Ring Position:
    position = robot.getPixelsPosition();

    //Moves to Shooting Position:
    startCoordinates[0] = 200.0;
    startCoordinates[1] = 760.0;
    endCoordinates[0] = 200.0;
    endCoordinates[1] = 460.0;
    robot.controller.runToPosition("forward", 1, startCoordinates, endCoordinates, robot.mainPower);

    /* Shooting */

    //Turns Robot and Shoots:
    robot.controller.turnGyro("left", 70.0, robot.mainPower);
    robot.controller.turnGyro("right", 20.0, robot.mainPower);

    /* Wobble Goal and Park */

    //Checks the Case:
    if (position == 1) {
      //Turns to the Wobble:
      robot.controller.turnGyro("right", 40.0, robot.mainPower);

      //Moves to Drop:
      startCoordinates[0] = endCoordinates[0];
      startCoordinates[1] = endCoordinates[1];
      endCoordinates[0] = 100.0;
      endCoordinates[1] = 360.0;
      robot.controller.runToPosition("forward", 2, startCoordinates, endCoordinates, robot.mainPower);
    }

    else if (position == 2) {
      //Moves to Drop:
      startCoordinates[0] = endCoordinates[0];
      startCoordinates[1] = endCoordinates[1];
      endCoordinates[0] = 200.0;
      endCoordinates[1] = 290.0;
      robot.controller.runToPosition("forward", 2, startCoordinates, endCoordinates, robot.mainPower);

      //Drops Wobble:
      robot.mechanisms.automateArm();

      //Moves to Park:
      startCoordinates[0] = endCoordinates[0];
      startCoordinates[1] = endCoordinates[1];
      endCoordinates[0] = 280.0;
      endCoordinates[1] = 350.0;
      robot.controller.runToPosition("backward", 2, startCoordinates, endCoordinates, robot.fastPower);
    }

    else {
      //Turns to the Wobble:
      robot.controller.turnGyro("right", 10.0, robot.mainPower);

      //Moves to Drop:
      startCoordinates[0] = endCoordinates[0];
      startCoordinates[1] = endCoordinates[1];
      endCoordinates[0] = 120.0;
      endCoordinates[1] = 160.0;
      robot.controller.runToPosition("forward", 2, startCoordinates, endCoordinates, robot.mainPower);

      //Drops Wobble:
      robot.mechanisms.automateArm();

      //Moves to Park:
      startCoordinates[0] = endCoordinates[0];
      startCoordinates[1] = endCoordinates[1];
      endCoordinates[0] = 210.0;
      endCoordinates[1] = 350.0;
      robot.controller.runToPosition("backward", 2, startCoordinates, endCoordinates, robot.fastPower);
    }

    /* Stop */

    //Status Update:
    telemetry.addData("Status", "Stopped");
    telemetry.addData("Position", position);
    telemetry.update();
  }
}
