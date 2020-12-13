package org.firstinspires.ftc.teamcode.Systems.Core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
  private static double Kc = 0.0;
  private static double errorMargin = 0;
  private static int stopPID = 0;

  /* CONTROLLER SETUP METHODS */

  //Constructor:
  public Controller() {
    super();
  }

  //Control Initialization Method:
  public static void initControl(double proportional, double integral, double derivative,
    double coefficient, double margin, int stop) {
    //Sets the Controller Values:
    Kp = proportional;
    Ki = integral;
    Kd = derivative;
    Kc = coefficient;
    errorMargin = margin;
    stopPID = stop;
  }

  /* CONTROLLER PID METHODS */

  //Control PID Method:
  public static void applyPID(DcMotor motor) {
    //Gets the Values:
    double error = motor.getCurrentPosition();
    double lastError = 0;
    double integral = 0;
    int count = 0;

    //Loops for Power:
    mainLoop: while (Math.abs(error) <= errorMargin && count < stopPID) {
      //Gets the Errors:
      error = (motor.getCurrentPosition() - motor.getTargetPosition());
      double deltaError = (lastError - error);
      integral += (deltaError * time.time());
      double derivative = (deltaError / time.time());

      //Gets the PID Values:
      double P = (Kp * error);
      double I = (Ki * integral);
      double D = (Kd * derivative);
      double power = (P + I + D);

      //Sets the Power and Resets:
      motor.setPower(power);
      error = lastError;
      time.reset();

      count++;
    }
  }

  //Controller Control Method:
  public static void applyController(DcMotor motor) {
    //Gets Error Values:
    int initialDifference = Math.abs(Math.abs(motor.getTargetPosition())
        - Math.abs(motor.getCurrentPosition()));
    int count = 0;

    //Loops for Power:
    mainLoop: while (controlError(motor.getCurrentPosition(), motor.getTargetPosition(),
      initialDifference) <= errorMargin && count < stopPID) {
      //Sets the Motor Power:
      double power = controlPower(motor, initialDifference);
      motor.setPower(power);

      count++;
    }
  }

  /* CONTROL ALGORITHM METHODS */

  //Control Power Calculation Method (DcMotor):
  public static double controlPower(DcMotor motor, int initialDifference) {
    //Gets the Values:
    int current = motor.getCurrentPosition();
    int target = motor.getTargetPosition();

    //Calculates and Returns the Power:
    double error = controlError(current, target, initialDifference);
    double controlledPower = Math.abs(error * Kc);
    return controlledPower;
  }

  //Control Error Method:
  public static double controlError(int current, int target, int initialDifference) {
    //Calculates and Returns Error:
    double error = Math.abs(Math.abs(target) - Math.abs(current));
    error /= Math.abs(initialDifference);
    return error;
  }
}