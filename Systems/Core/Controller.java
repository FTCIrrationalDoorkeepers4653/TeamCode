package org.firstinspires.ftc.teamcode.Systems.Core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class Controller extends Positions {
  /* CONTROLLER VARIABLES */

  //Controller Objects:
  private static Robot robot = new Robot();
  private static ElapsedTime time = new ElapsedTime();

  //Controller Variables:
  private static double Kp = 0.0;
  private static double Ki = 0.0;
  private static double Kd = 0.0;

  //Constructor:
  public Controller() {
    super();
  }

  /* CONTROLLER CONTROL METHODS */

  //Controller Initialization Method:
  public static void initController(double proportional, double integral, double derivative) {
    //Sets the Controller Values:
    Kp = proportional;
    Ki = integral;
    Kd = derivative;
  }

  //Controller Arm Power Method:
  public static void applyControlMotorPower(DcMotor motor, double power) {
    int initialDifference = Math.abs(Math.abs(motor.getTargetPosition())
      - Math.abs(motor.getCurrentPosition()));
    double error = controlError(motor.getCurrentPosition(),
      motor.getTargetPosition(), initialDifference);
    motor.setPower(power);
  }

  /* CONTROL ALGORITHM METHODS */

  //Controller Error Method:
  public static double controlError(int current, int target, int initialDifference) {
    //Calculates and Returns Error:
    double error = Math.abs(Math.abs(target) - Math.abs(current));
    error /= Math.abs(initialDifference);
    return error;
  }
}