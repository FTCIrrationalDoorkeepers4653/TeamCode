package org.firstinspires.ftc.teamcode.Programs.TeleOp;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Systems.Movement.IDKRobot;

@TeleOp(name = "Vuforia_Test")
//@Disabled
public class Vuforia_Test extends LinearOpMode {
  /* TEST VARIABLES */

  //Objects:
  IDKRobot robot = new IDKRobot();
  private ElapsedTime runtime = new ElapsedTime();

  //Variables:
  int position = 1;
  boolean runType = false;

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

    //Loops Until Stop:
    while (!isStopRequested()) {
      //Telemetry Data:
      telemetry.addData("Position", position);
      telemetry.update();
    }

    //Status Update:
    telemetry.addData("Status", "Stopped");
    telemetry.update();
  }
}

