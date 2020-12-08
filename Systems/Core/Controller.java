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
  private static double errorMargin = 0;
  private static int stopPID = 0;

  //Controller Interface Variables:
  private static double TPR = 0.0;
  private static double RPM = 0.0;

  /* CONTROLLER SETUP METHODS */

  //Constructor:
  public Controller() {
    super();
  }

  //Control Initialization Method:
  public static void initControl(double proportional, double integral, double derivative,
    double margin, int stop) {
    //Sets the Controller Values:
    Kp = proportional;
    Ki = integral;
    Kd = derivative;
    errorMargin = margin;
    stopPID = stop;
  }

  //Control Motor Interface Setup Method:
  public static void setupControlInterface(double ticksPerRev, double maxRPM) {
    //Sets the Motor Interface Variables:
    TPR = ticksPerRev;
    RPM = maxRPM;
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

  //Control PID Method (Secondary):
  public static void applyPID(DcMotorEx motor) {
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

  /* CONTROLLER CONTROL METHODS */

  //Control Power Method:
  public static void applyControlMotorPower(DcMotor motor, double power) {
    //Gets and Sets the Motor Power:
    double controlledPower = controlPower(motor, power);
    motor.setPower(power);
  }

  //Control Ex Power Method:
  public static void applyControlMotorPowerEx(DcMotorEx motor, double ticks) {
    //Gets and Sets the Motor Power:
    double power = controlTicks(ticks, TPR, RPM);
    double controlledPower = controlPowerEx(motor, power);
    motor.setVelocity(ticks);
  }

  /* CONTROL ALGORITHM METHODS */

  //Control Power Calculation Method (DcMotor):
  public static double controlPower(DcMotor motor, double power) {
    //Gets the Values:
    int initialDifference = Math.abs(Math.abs(motor.getTargetPosition())
      - Math.abs(motor.getCurrentPosition()));
    int current = motor.getCurrentPosition();
    int target = motor.getTargetPosition();

    //Calculates and Returns the Power:
    double error = controlError(current, target, initialDifference);
    double controlledPower = (error * power);
    return controlledPower;
  }

  //Control Power Calculation Method (DcMotorEx):
  public static double controlPowerEx(DcMotorEx motor, double power) {
    //Gets the Values:
    int initialDifference = Math.abs(Math.abs(motor.getTargetPosition())
        - Math.abs(motor.getCurrentPosition()));
    int current = motor.getCurrentPosition();
    int target = motor.getTargetPosition();

    //Calculates and Returns the Power:
    double error = controlError(current, target, initialDifference);
    double controlledPower = (error * power);
    return controlledPower;
  }

  /* CONTROLLER CALCULATION METHODS */

  //Control Ticks Method:
  public static double controlTicks(double ticks, double ticksPerRev, double maxRPM) {
    //Gets the Values:
    double targetRPS = (ticks / ticksPerRev);
    double maxRPS = (maxRPM / 60.0);

    //Gets and Returns Power:
    double power = (targetRPS / maxRPS);
    return power;
  }

  //Control Error Method:
  public static double controlError(int current, int target, int initialDifference) {
    //Calculates and Returns Error:
    double error = Math.abs(Math.abs(target) - Math.abs(current));
    error /= Math.abs(initialDifference);
    return error;
  }
}