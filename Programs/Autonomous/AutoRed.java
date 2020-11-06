package org.firstinspires.ftc.teamcode.Programs.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Systems.Core.Robot;;

@Autonomous(name="AutoRed")
//@Disabled
public class AutoRed extends LinearOpMode {
  /* AUTO RED VARIABLES */

  //Robot Objects:
  private Robot robot = new Robot();

  //Movement Variables:
  private double startCoordinates[] = new double[2];
  private double endCoordinates[] = new double[2];
  private int position = 0;

  @Override
  public void runOpMode() {
    /* Initialization */

    //Status Updates:
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    //Hardware Initialization:
    robot.init(hardwareMap, true, true);
    robot.mechanisms.initMechanisms(hardwareMap);
    robot.mechanisms.ringCount = 3;

    //Waits for Start:
    waitForStart();

    /* Detection */

    //Gets the Ring Position:
    position = robot.getPixelsPosition();
    robot.mechanisms.automateFlywheel();

    //Moves to Shooting Position:
    startCoordinates[0] = 560.0;
    startCoordinates[1] = 760.0;
    endCoordinates[0] = 560.0;
    endCoordinates[1] = 460.0;
    robot.mechanisms.runToPosition("forward", 1, startCoordinates, endCoordinates, robot.mainPower);

    /* Shooting */

    //Turns Robot and Shoots:
    robot.mechanisms.turnGyro("right", 70.0, robot.mainPower);
    robot.mechanisms.automateShooter();

    //Turns Robot and Shoots:
    robot.mechanisms.turnGyro("right", 15.0, robot.mainPower);
    robot.mechanisms.automateShooter();

    //Turns Robot and Shoots Final:
    robot.mechanisms.turnGyro("right", 10.0, robot.mainPower);
    robot.mechanisms.automateShooter();
    robot.mechanisms.automateFlywheel();

    /* Wobble Goal and Park */

    //Checks the Case:
    if (position == 1) {
      //Turns to the Wobble:
      robot.mechanisms.turnGyro("left", 35.0, robot.mainPower);

      //Moves to Drop:
      startCoordinates[0] = endCoordinates[0];
      startCoordinates[1] = endCoordinates[1];
      endCoordinates[0] = 680.0;
      endCoordinates[1] = 360.0;
      robot.mechanisms.runToPosition("forward", 2, startCoordinates, endCoordinates, robot.mainPower);

      //Drops Wobble:
      robot.mechanisms.arm = 1;
      robot.mechanisms.automateArm(robot.mainPower);
    }

    else if (position == 2) {
      //Turns to the Wobble:
      robot.mechanisms.turnGyro("left", 75.0, robot.mainPower);

      //Moves to Drop:
      startCoordinates[0] = endCoordinates[0];
      startCoordinates[1] = endCoordinates[1];
      endCoordinates[0] = 560.0;
      endCoordinates[1] = 310.0;
      robot.mechanisms.runToPosition("forward", 2, startCoordinates, endCoordinates, robot.mainPower);

      //Drops Wobble:
      robot.mechanisms.arm = 1;
      robot.mechanisms.automateArm(robot.mainPower);

      //Moves to Park:
      startCoordinates[0] = endCoordinates[0];
      startCoordinates[1] = endCoordinates[1];
      endCoordinates[0] = 480.0;
      endCoordinates[1] = 330.0;
      robot.mechanisms.runToPosition("backward", 2, startCoordinates, endCoordinates, robot.mainPower);
    }

    else {
      //Turns to the Wobble:
      robot.mechanisms.turnGyro("left", 65.0, robot.mainPower);

      //Moves to Drop:
      startCoordinates[0] = endCoordinates[0];
      startCoordinates[1] = endCoordinates[1];
      endCoordinates[0] = 640.0;
      endCoordinates[1] = 160.0;
      robot.mechanisms.runToPosition("forward", 2, startCoordinates, endCoordinates, robot.mainPower);

      //Drops Wobble:
      robot.mechanisms.arm = 1;
      robot.mechanisms.automateArm(robot.mainPower);

      //Moves to Park:
      startCoordinates[0] = endCoordinates[0];
      startCoordinates[1] = endCoordinates[1];
      endCoordinates[0] = 550.0;
      endCoordinates[1] = 330.0;
      robot.mechanisms.runToPosition("backward", 2, startCoordinates, endCoordinates, robot.mainPower);
    }

    //Resets the Arm:
    robot.mechanisms.arm = 0;
    robot.mechanisms.automateArm(robot.mainPower);

    /* Stop */

    //Status Update:
    telemetry.addData("Status", "Stopped");
    telemetry.addData("Position", position);
    telemetry.update();
  }
}