package org.firstinspires.ftc.teamcode.Systems.Detectors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Systems.Movement.IDKRobot;

@TeleOp(name = "Vuforia_Test")
//@Disabled
public class Vuforia_Test extends LinearOpMode {
  //Objects:
  IDKRobot robot = new IDKRobot();

  //Variables (w/ Default Values):
  int position = 1;

  //Runtime Object:
  private ElapsedTime runtime = new ElapsedTime();

  @Override
  public void runOpMode() {
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    //Hardware INITS:
    robot.init(hardwareMap, false);

    waitForStart();
    runtime.reset();

    //Loop for Detector:
    while (!isStopRequested()) {
      //Gets the Position:
      position = robot.getPixelsPosition();

      //Telemetry Data:
      telemetry.addData("Position: ", position);
      telemetry.update();
    }
  }
}

