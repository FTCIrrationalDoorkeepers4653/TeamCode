package org.firstinspires.ftc.teamcode.Systems.Core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class GamePad {
  /* GAMEPAD MAIN VARIABLES */

  //Main GamePad Variables:
  private Gamepad gamepad = new Gamepad();
  private Robot robot = new Robot();

  //Control Button Variables:
  public boolean aButton = false;
  public boolean bButton = false;
  public boolean xButton = false;
  public boolean yButton = false;

  //Joystick Variables:
  public double leftX = 0.0;
  public double leftY = 0.0;
  public double rightX = 0.0;
  public double rightY = 0.0;
  public boolean leftJoyButton = false;
  public boolean rightJoyButton = false;

  //Trigger Variables:
  public double leftTrigger = 0.0;
  public double rightTrigger = 0.0;
  public boolean leftBumper = false;
  public boolean rightBumper = false;

  //D-Pad and Button Variables:
  public boolean dPadUp = false;
  public boolean dPadDown = false;
  public boolean dPadLeft = false;
  public boolean dPadRight = false;

  //Control Buttons Tracker Variables:
  private boolean wasAPressed = false;
  private boolean wasBPressed = false;
  private boolean wasXPressed = false;
  private boolean wasYPressed = false;

  /* GAMEPAD SETUP */

  //Constructor:
  public GamePad(Gamepad gamepad) {
    super();
    this.gamepad = gamepad;
  }

  //Sets the GamePad Values:
  public void setGamePad() {
    //Control Buttons:
    aButton = gamepad.a;
    bButton = gamepad.b;
    xButton = gamepad.x;
    yButton = gamepad.y;

    //Joysticks:
    leftX = gamepad.left_stick_x;
    leftY = gamepad.left_stick_y;
    rightX = gamepad.right_stick_x;
    rightY = gamepad.right_stick_y;
    leftJoyButton = gamepad.left_stick_button;
    rightJoyButton = gamepad.right_stick_button;

    //Triggers:
    leftTrigger = gamepad.left_trigger;
    rightTrigger = gamepad.right_trigger;
    leftBumper = gamepad.left_bumper;
    rightBumper = gamepad.right_bumper;

    //D-Pad:
    dPadUp = gamepad.dpad_up;
    dPadDown = gamepad.dpad_down;
    dPadLeft = gamepad.dpad_left;
    dPadRight = gamepad.dpad_right;
  }

  /* GAMEPAD METHODS */

  //A Button Released:
  public boolean isAReleased() {
    //Checks the Case:
    if (aButton) {
      wasAPressed = true;
    }

    //Checks the Case:
    if (wasAPressed && !aButton) {
      //Returns the Value:
      wasAPressed = false;
      return true;
    } else {
      //Returns the Value:
      return false;
    }
  }

  //B Button Released:
  public boolean isBReleased() {
    //Checks the Case:
    if (bButton) {
      wasBPressed = true;
    }

    //Checks the Case:
    if (wasBPressed && !bButton) {
      //Returns the Value:
      wasBPressed = false;
      return true;
    } else {
      //Returns the Value:
      return false;
    }
  }

  //X Button Released:
  public boolean isXReleased() {
    //Checks the Case:
    if (xButton) {
      wasXPressed = true;
    }

    //Checks the Case:
    if (wasXPressed && !xButton) {
      //Returns the Value:
      wasXPressed = false;
      return true;
    } else {
      //Returns the Value:
      return false;
    }
  }

  //Y Button Released:
  public boolean isYReleased() {
    //Checks the Case:
    if (yButton) {
      wasYPressed = true;
    }

    //Checks the Case:
    if (wasYPressed && !yButton) {
      //Returns the Value:
      wasYPressed = false;
      return true;
    } else {
      //Returns the Value:
      return false;
    }
  }
}
