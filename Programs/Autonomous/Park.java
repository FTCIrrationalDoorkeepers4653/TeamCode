package org.firstinspires.ftc.teamcode.Programs.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Systems.Movement.Robot;

@Autonomous(name = "Park")
//@Disabled
public class Park extends LinearOpMode {
  /* PARK VARIABLES */

  //Robot Objects:
  Robot robot = new Robot();

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

    /* Parks */

    //Grabs the Wobble:
    //robot.operateClaw("close");
    //idle();
    //sleep(1000);
    //robot.finishRun();
    //idle();

    //Parks the Robot:
    robot.positions.findPath(motions, startCoordinates, endCoordinates, turnFirst);
    robot.positions.moveToPosition(motions, robot.slowPower, turnFirst);

    /* End */

    //Status Update:
    telemetry.addData("Status", "Stopped");
    telemetry.update();
  }
}
