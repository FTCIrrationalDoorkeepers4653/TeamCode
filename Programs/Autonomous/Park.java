package org.firstinspires.ftc.teamcode.Programs.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Systems.Movement.IDKRobot;
import org.firstinspires.ftc.teamcode.Systems.Movement.PositionMovement;

@Autonomous(name = "Park")
//@Disabled
public class Park extends LinearOpMode {
  /* PARK VARIABLES */

  //Robot Objects:
  IDKRobot robot = new IDKRobot();
  PositionMovement positionMovement = new PositionMovement();
  HardwareMap hwMap;
  ElapsedTime runtime = new ElapsedTime();

  //Park Variables:
  double startCoordinates[] = {660.0, 788.0},
    endCoordinates[] = {660.0, 396.0};
  String motions[] = {"forward", "nothing"};
  boolean turnFirst = false, runType = true;

  /* OPMODE METHODS */

  @Override
  public void runOpMode() {
    //Initializes the Robot:
    robot.init(hwMap, runType);

    //Wait for Start:
    waitForStart();
    runtime.reset();

    //Grabs the Wobble:
    robot.operateClaw("close");
    idle();
    sleep(2000);
    robot.finishRun();
    idle();

    //Parks the Robot:
    positionMovement.findPath(motions, startCoordinates, endCoordinates, turnFirst);
    positionMovement.moveToPosition(motions, robot.mainPower, turnFirst);

    //Disables Vuforia:
    robot.imageInit.disableVuforia();
  }
}
