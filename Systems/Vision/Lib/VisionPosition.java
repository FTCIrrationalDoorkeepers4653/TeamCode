package org.firstinspires.ftc.teamcode.Systems.Vision.Lib;

import android.graphics.Bitmap;

import org.firstinspires.ftc.teamcode.Systems.Vision.Lib.Camera;

import java.util.ArrayList;

@SuppressWarnings("unused")
public class VisionPosition extends Camera {
  /* VISION POSITIONING VARIABLES */

  //Positioning Data (w/ Defaults):
  private static int referencePixel[] = new int[2], centerPixel[] = new int[2];
  private static double referenceDistanceInches = 0, cameraOffsetX = 0, cameraOffsetY = 0;

  //Vision Positioning Output Variables (w/ Defaults):
  private static ArrayList<Double> alignX = new ArrayList<Double>();
  private static ArrayList<Double> alignY = new ArrayList<Double>();

  /* VISION POSITIONING SETUP METHODS */

  //Constructor:
  public VisionPosition() {
    super();
  }

  //Initialize Vision Positioning:
  public static void initVisionPosition(int reference[], double referenceDistance, double camOffsetX,
    double camOffsetY) {
    //Sets Distances and Offesets:
    referenceDistanceInches = referenceDistance;
    cameraOffsetX = camOffsetX;
    cameraOffsetX = camOffsetY;

    //Checks the Case:
    if (reference.length == 2) {
      //Sets the Reference Pixel:
      referencePixel = reference;
    }

    else {
      //Sets the Reference Pixel Coordinates:
      referencePixel[0] = 0;
      referencePixel[1] = 0;
    }
  }

  /* VISION POSITIONING METHODS */

  //Image Analysis Pipeline Method:
  public static void positionImage(double resizeRatio, int lightingMargin[], int blobThreshold) {
    //Gets the Center Image from Frame:
    Bitmap image = getImage(resizeRatio);
    int rgb[][] = getBitmapRGB(image, 0, 0, image.getWidth(), image.getHeight());
    int coordinates[][] = getCoordinates(rgb, lightingMargin, blobThreshold);
    int center[] = getCenter(image);

    //Loop Variable:
    int turns = 0;

    //Loops through Array:
    mainLoop: while (turns < coordinates[0].length) {


      turns++;
    }
  }

  //Center of Image Method:
  public static int[] getCenter(Bitmap image) {
    //Gets the Center Coordinates:
    int x = (image.getWidth() / 2);
    int y = (image.getHeight() / 2);

    //Formats and Returns Array:
    int center[] = {x, y};
    centerPixel = center;
    return center;
  }

  //Get Ray Coordinates Method:
  public static int[] getRayCast(int coordinate[]) {
    //Main RayCast Coordinate:
    int rayCoordinate[] = new int[2];
    rayCoordinate[1] = referencePixel[1];

    //Gets the Initial Slope and X-Coordinate:
    double initialSlope = ((Math.abs(coordinate[1] - centerPixel[1])) / (Math.abs(coordinate[0] - centerPixel[0])));
    rayCoordinate[0] = (int)(((Math.abs(rayCoordinate[1] - coordinate[1])) / initialSlope) + coordinate[0]);
    return rayCoordinate;
  }

  /* VISION POSITION UTILITY METHODS */

  //Get Alignment X Distance Method:
  public static ArrayList<Double> getAlignX() {
    //Returns the X Alignments:
    return alignX;
  }

  //Get Alignment Y Distance Method:
  public static ArrayList<Double> getAlignY() {
    //Returns the Y Alignments:
    return alignY;
  }
}
