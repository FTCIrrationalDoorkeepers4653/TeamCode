package org.firstinspires.ftc.teamcode.Programs.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Systems.Movement.IDKRobot;
import org.firstinspires.ftc.teamcode.Systems.Movement.PositionMovement;

@Autonomous(name="AutoRed")
//@Disabled
public class AutoRed extends LinearOpMode {
  /* AUTO RED VARIABLES */

  //Robot Objects:
  IDKRobot robot = new IDKRobot();
  PositionMovement positionMovement = new PositionMovement();
  ElapsedTime runtime = new ElapsedTime();

  //Movement Variables:
  double startCoordinates[] = {513.0, 760.0},
    endCoordinates[] = {513.0, 377.0};
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
    runtime.reset();

    /* Moves to Pivot */

    //Grabs the Wobble:
    //robot.operateClaw("close");
    //idle();
    //sleep(1000);
    //robot.finishRun();
    //idle();

    //Gets the Ring Position:
    position = robot.getPixelsPosition();

    //Moves to the Pivot:
    positionMovement.findPath(motions, startCoordinates, endCoordinates, turnFirst);
    positionMovement.moveToPosition(motions, robot.mainPower, turnFirst);

    /* Moves to Drop Wobble Goal */

    //Sets the Coordinates:
    motions[0] = "right";
    motions[1] = "forward";
    startCoordinates = endCoordinates;
    turnFirst = true;

    //Checks the Case:
    if (position == 1) {
      //Sets the End Coordinates:
      endCoordinates[0] = 640.0;
      endCoordinates[1] = 344.0;
    }

    else if (position == 2) {
      //Sets the End Coordinates:
      endCoordinates[0] = 548.0;
      endCoordinates[1] = 251.0;
    }

    else {
      //Sets the End Coordinates:
      endCoordinates[0] = 671.0;
      endCoordinates[1] = 122.0;
    }

    //Moves to Position:
    positionMovement.findPath(motions, startCoordinates, endCoordinates, turnFirst);
    positionMovement.moveToPosition(motions, robot.mainPower, turnFirst);

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
    endCoordinates[0] = 513.0;
    endCoordinates[1] = 377.0;
    turnFirst = false;

    //Moves to Position:
    positionMovement.findPath(motions, startCoordinates, endCoordinates, turnFirst);
    positionMovement.moveToPosition(motions, robot.mainPower, turnFirst);

    /* Turn to Shoot */

    //Turns:
    positionMovement.turnGyro("right", 45.0, robot.mainPower);

    /* Shoot */

    /* Park */

    //Sets the Coordinates:
    motions[0] = "forward";
    motions[1] = "nothing";
    startCoordinates = endCoordinates;
    endCoordinates[0] = 475.0;
    endCoordinates[1] = 345.0;

    //Moves to Position:
    positionMovement.findPath(motions, startCoordinates, endCoordinates, turnFirst);
    positionMovement.moveToPosition(motions, robot.mainPower, turnFirst);

    /* End */

    //Status Update:
    telemetry.addData("Status: ", "Stopped");
    telemetry.update();
  }
}
