package org.firstinspires.ftc.teamcode.Systems.Core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class Mechanisms extends Positions {
  /* MECHANISMS SETUP VARIABLES */

  //Robot Object:
  private static Robot robot = new Robot();

  //Motor Mechanisms:
  public static DcMotor slideMotor;
  public static DcMotor carouselMotor;

  //Servo Mechanisms:
  public static Servo basketServo;
  public static Servo intakeServo;
  public static Servo levelsServo;

  /* MECHANISMS ARM CONTROL VARIABLES */

  //Linear Slide Variables:
  public static int slideMovement = -655;
  public static int slide = 0;

  //Bucket Variables:
  public static double basketUpPosition = 0.2;
  public static double basketDownPosition = 0.0;
  public static int basket = 0;

  //Intake Servo Variables:
  public static double intakeStartPosition = 1.0;
  public static double intakeEndPosition = 0.0;
  public static int intake = 0;

  //Positions for the levels of the shipping hub
  public static double levelsInitPosition = 0.3;
  public static double levelsHigh = 0.2;
  public static double levelsMid = 0.1;
  public static double levelsLow = 0.0;
  public static int levels = 0;

  /* MECHANISM SHOOTER CONTROL VARIABLES */

  //Mechanism Flywheel Variables:
  public static int shooter = 0;

  //TeleOp Automation Variables:
  public static int everything = 0;

  /* MECHANISMS INITIALIZATION METHODS */
  //Constructor:
  public Mechanisms() {
    super();
  }

  //Initializes the Mechanisms:
  public static void initMechanisms(HardwareMap hardwareMap) {
    /* Initialization */

    //Motor Mechanism Maps:
    slideMotor = hardwareMap.dcMotor.get("slideMotor");
    carouselMotor = hardwareMap.get(DcMotor.class, "carouselMotor");

    //Servo Mechanism Maps:
    basketServo = hardwareMap.servo.get("basketServo");
    intakeServo = hardwareMap.servo.get("intakeServo");
    levelsServo = hardwareMap.servo.get("levelsServo");

    //Servo Mechanism Setup:
    basketServo.setPosition(basketUpPosition);
    intakeServo.setPosition(intakeStartPosition);
    levelsServo.setPosition(levelsInitPosition);

    /* Setup */

    //Mechanism Motors Behavior:
    slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    //Mechanism Motors Direction:
    slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    carouselMotor.setDirection(DcMotor.Direction.REVERSE);

    //Mechanism Motors Encoders Reset:
    carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    //Mechanism Motors Encoders:
    slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    carouselMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    //Mechanism Motors Powers:
    slideMotor.setPower(robot.zeroPower);
    carouselMotor.setPower(robot.zeroPower);

    slide = 0;
    intake = 0;
    levels = 0;
    basket = 0;
    shooter = 0;
    everything = 0;
  }

  /* AUTONOMOUS OPERATION METHODS */

  public void automateFlywheel(int on) {
    //1 makes it turn off, 0 makes it turn off
    shooter = on;
    operateFlywheel();
  }

  //Automates the basket
  public void automateBasket(int up) {
    //0 makes it go down, 1 makes it go up
    basket = up;
    operateBasket();
  }

  //Automates the linear slides
  public void automateSlides(int up) {
    //if up = 0 then put the slides up; otherwise put them down
    slide = up;
    operateSlides();
    sleep(3830);
  }

  //Automates the Level:
  public void automateLevels(int level) {
    levels = level;
    operateLevels();
  }

  /* MECHANISM OPERATION METHODS */

  //Operates the Flywheel:
  public void operateFlywheel() {
    //Checks the Case:
    if (shooter == 0) {
      //Sets the Motor:
      carouselMotor.setPower(robot.zeroPower);
      shooter++;
    }

    else if (shooter == 1) {
      //Sets the Motor:
      carouselMotor.setPower(-robot.flywheelPower);
      sleep(3000);
      shooter--;
    }
  }

  //Operates the Intake Arm:
  public void operateIntake() {
    //Checks the Case:
    if (intake == 0) {
      //Runs the Intake:
      intakeServo.setPosition(intakeEndPosition);
      intake++;
    }

    else if (intake == 1) {
      //Runs the Intake:
      intakeServo.setPosition(intakeStartPosition);
      intake--;
    }
  }

  //method to operate the basket and dump the game element into the top compartment
  public void operateBasket() {
    if (basket == 0) {
      basketServo.setPosition(basketDownPosition);
      sleep(1000);
      basket++;
    }

    else if (basket == 1) {
      basketServo.setPosition(basketUpPosition);
      sleep(1000);
      basket--;
    }
  }

  //Linear slide operation method
  public void operateSlides() {
    //Go up
    if (slide == 0) {
      automateLevels(2);
      slideMotor.setTargetPosition(slideMotor.getCurrentPosition() + slideMovement);
      slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      slideMotor.setPower(robot.gyroPower);
//      sleep(3830);
      slide++;
    }

    //go down
    else if (slide == 1) {
      automateBasket(1);
      automateLevels(2);
      slideMotor.setTargetPosition(slideMotor.getCurrentPosition() - slideMovement);
      slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      slideMotor.setPower(robot.gyroPower);
//      sleep(6360);
      slide--;
      automateBasket(0);
    }
  }

  public void operateLevels() {
    //Checks the Case:
    if (levels == 0) {
      //Runs the Intake:
      levelsServo.setPosition(levelsHigh);
      sleep(125);
      levels++;
    }

    else if (levels == 1) {
      //Runs the Intake:
      levelsServo.setPosition(levelsMid);
      sleep(125);
      levels++;
    }

    else if (levels == 2) {
      //Runs the Intake:
      levelsServo.setPosition(levelsLow);
      sleep(125);
      levels -= 2;
    }
  }

  public void operateEverything() {
    if (everything == 0) {
      //Hit the intake
      intakeServo.setPosition(intakeEndPosition);
      sleep(800);
      intakeServo.setPosition(intakeStartPosition);
      //Make the basket go up
      automateBasket(1);
      //if the object doesn't go into the basket the fine tune from here
      everything++;
    }

    //if it does work though press the same button to continue
    else if (everything == 1) {
      automateLevels(2);
      //raise the slides
      operateSlides();
//      automateSlides(0);
      everything++;
    }

    //when lined up
    else if (everything == 2) {
      automateBasket(0);
      automateBasket(1);
      automateLevels(2);
      operateSlides();
//      automateSlides(1);
      automateBasket(0);
      everything -= 2;
    }
  }
}