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
  ElapsedTime runtime = new ElapsedTime();

  //Movement Variables:
  double startCoordinates[] = {652.0, 760.0},
    endCoordinates[] = {652.0, 375.0};
  String motions[] = {"forward", "nothing"};
  boolean turnFirst = false, runType = true;

  /* OPMODE METHODS */

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

    /* Parks */

    //Grabs the Wobble:
    //robot.operateClaw("close");
    //idle();
    //sleep(1000);
    //robot.finishRun();
    //idle();

    //Parks the Robot:
    positionMovement.findPath(motions, startCoordinates, endCoordinates, turnFirst);
    positionMovement.moveToPosition(motions, robot.slowPower, turnFirst);

    /* End */

    //Status Update:
    telemetry.addData("Status", "Stopped");
    telemetry.update();
  }
}
