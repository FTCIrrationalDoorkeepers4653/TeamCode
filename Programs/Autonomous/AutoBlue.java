package org.firstinspires.ftc.teamcode.Programs.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Systems.Core.Robot;;

@Autonomous(name="AutoBlue")
//@Disabled
public class AutoBlue extends LinearOpMode {
  /* AUTO BLUE VARIABLES */

  //Robot Objects:
  Robot robot = new Robot();

  //Movement Variables:
  double startCoordinates[] = {660.0, 788.0},
    endCoordinates[] = {660.0, 396.0};
  String motions[] = {"forward", "nothing"};
  boolean turnFirst = false, runType = true;

  //Autonomous Variables:
  int position = 1;

  @Override
  public void runOpMode() {
    //Status Updates:
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    //Hardware Initialization:
    robot.init(hardwareMap, runType);

    //Waits for Start:
    waitForStart();

    /* Moves to Pivot */

    //Grabs the Wobble:
    //robot.operateClaw("close");
    //idle();
    //sleep(1000);
    //robot.finishRun();
    //idle();

    //Gets the Ring Position:
    position = robot.getPixelsPosition();
    idle();
    sleep(2000);
    robot.finishRun();
    idle();

    //Moves to the Pivot:
    robot.positions.findPath(motions, startCoordinates, endCoordinates, turnFirst);
    robot.positions.moveToPosition(motions, robot.mainPower, turnFirst);

    /* Moves to Drop Wobble Goal */

    //Sets the Coordinates:
    motions[0] = "left";
    motions[1] = "forward";
    startCoordinates = endCoordinates;
    turnFirst = true;

    //Checks the Case:
    if (position == 1) {
      //Sets the End Coordinates:
      endCoordinates[0] = 120.0;
      endCoordinates[1] = 344.0;
    }

    else if (position == 2) {
      //Sets the End Coordinates:
      endCoordinates[0] = 212.0;
      endCoordinates[1] = 251.0;
    }

    else {
      //Sets the End Coordinates:
      endCoordinates[0] = 89.0;
      endCoordinates[1] = 122.0;
    }

    //Moves to Position:
    robot.positions.findPath(motions, startCoordinates, endCoordinates, turnFirst);
    robot.positions.moveToPosition(motions, robot.mainPower, turnFirst);

    /* Unlatch */

    //Drops Wobble:
    robot.operateClaw("open");
    idle();
    sleep(2000);
    robot.finishRun();
    idle();

    /* Move Back */

    //Sets the Coordinates:
    motions[0] = "backward";
    motions[1] = "nothing";
    startCoordinates = endCoordinates;
    endCoordinates[0] = 247.0;
    endCoordinates[1] = 377.0;
    turnFirst = false;

    //Moves to Position:
    robot.positions.findPath(motions, startCoordinates, endCoordinates, turnFirst);
    robot.positions.moveToPosition(motions, robot.mainPower, turnFirst);

    /* Turn to Shoot */

    //Turns:
    robot.positions.turnGyro("left", -45.0, robot.mainPower);

    /* Shoot */

    /* Park */

    //Sets the Coordinates:
    motions[0] = "forward";
    motions[1] = "nothing";
    startCoordinates = endCoordinates;
    endCoordinates[0] = 285.0;
    endCoordinates[1] = 345.0;

    //Moves to Position:
    robot.positions.findPath(motions, startCoordinates, endCoordinates, turnFirst);
    robot.positions.moveToPosition(motions, robot.mainPower, turnFirst);

    /* End */

    //Status Update:
    telemetry.addData("Status", "Stopped");
    telemetry.update();
  }
}
