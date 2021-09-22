package org.firstinspires.ftc.teamcode.Systems.Vision;

import android.graphics.Bitmap;
import java.util.ArrayList;
import lib.Analyze;
import lib.ImageCorrection;
import lib.ImageProcessing;
import lib.Positioning;

@SuppressWarnings("unused")
public class Vision extends Analyze {
  /* VISION VARIABLES */

  //Vision Objects:
  private static Positioning positioning = new Positioning();
  private static ImageProcessing processing = new ImageProcessing();
  private static ImageCorrection correction = new ImageCorrection();

  /* SETUP METHODS */

  //Constructor:
  public Vision() {
    super();
  }

  //Detector Setup:
  public static void initDetector(String detectorName, int rgb[]) {
    try {
      //Detector Setup:
      initDetector(detectorName, rgb[0], rgb[1], rgb[2]);
    }

    catch (Exception e) {
      e.printStackTrace();
    }
  }

  //Vision Positioning Setup:
  public static void initPositioning(int width, int height, double ratioDist, double ratioOffset,
    double offsetX, double offsetY) {
    try {
      //Positioning Setup:
      positioning.initVisionPosition(width, height, ratioDist, ratioOffset, offsetX, offsetY);
    }

    catch (Exception e) {
      e.printStackTrace();
    }
  }

  /* RGB METHODS */

  //Gets the Array of RGB Values from an Image:
  public static int[][] getBitmapRGB(Bitmap image, int startX, int startY, int width, int height) {
    //Main RGB Array (w/ Default):
    int[][] rgbValues = new int[width][height];

    if (image != null && startX + width <= image.getWidth() && startY + height <= image.getHeight()) {
      //Gets RGB Values Array from Bitmap:
      int turnsWidth = startX;

      //Loops through Array:
      mainLoop: while (turnsWidth < startX + width) {
        //Loop Counter:
        int turnsHeight = startY;

        //Loops through Array:
        secondLoop: while (turnsHeight < startY + height) {
          //Sets the RGB Array Values:
          rgbValues[turnsWidth - startX][turnsHeight - startY] = image.getPixel(turnsWidth, turnsHeight);

          turnsHeight++;
        }

        turnsWidth++;
      }
    }

    //Returns the Obtained RGB Array:
    return rgbValues;
  }

  //Bitmap from RGB Method:
  public static Bitmap rgbBitmap(int rgb[][]) {
    //Creates Bitmap:
    Bitmap image = Bitmap.createBitmap(rgb.length, rgb[0].length, Bitmap.Config.RGB_565);

    //Loop Variable:
    int turnsWidth = 0;

    //Loops through Array:
    mainLoop: while (turnsWidth < rgb.length) {
      //Loop Variable:
      int turnsHeight = 0;

      //Loops through Array:
      secondLoop: while (turnsHeight < rgb[0].length) {
        //Sets the Pixel:
        image.setPixel(turnsWidth, turnsHeight, rgb[turnsWidth][turnsHeight]);

        turnsHeight++;
      }

      turnsWidth++;
    }

    //Returns the Image:
    return image;
  }

  /* DETECTOR METHODS */

  //RGB Comparison Method:
  public static int[] getAverageRGBValues(int rgbValues[][]) {
    //Main Array:
    int localAverage[] = new int[3];

    try {
      //Computes the Average:
      localAverage = averageRGBValues(rgbValues);
    }

    catch (Exception e) {
      e.printStackTrace();
    }

    //Returns RGB Average:
    return localAverage;
  }

  //Detection Boolean Method:
  public static boolean detect(int[][] rgbValues, int[] lightingMargin, int pixelCount) {
    //Main Boolean:
    boolean mainBool = false;

    try {
      //Finds the Value Number of Pixels and Sets Boolean:
      int binaryValues[][] = binaryDetector(rgbValues, lightingMargin);
      mainBool = getBodyBoolean(binaryValues, pixelCount);
    }

    catch (Exception e) {
      e.printStackTrace();
    }

    //Returns the Found Boolean:
    return mainBool;
  }

  //Detection Blob Method:
  public static int detectBlobs(int[][] rgbValues, int[] lightingMargin, int distanceThreshold) {
    //Main Blob Count (w/ Default):
    int mainCount = 0;

    try {
      //Finds the Value Number of Pixels and Sets Boolean:
      int binaryValues[][] = binaryDetector(rgbValues, lightingMargin);
      mainCount = getBodyBlobs(binaryValues, distanceThreshold);
    }

    catch (Exception e) {
      e.printStackTrace();
    }

    //Returns the Blob Count:
    return mainCount;
  }

  /* PIXEL COUNT METHODS */

  //Pixel Detection Method:
  public static int detectPixelCount(int[][] rgbValues, int[] lightingMargin, int pixels) {
    //Main Pixel Count Variable (w/ Default):
    int pixelCount = 0;

    try {
      //Gets the Boolean Pixel Count:
      boolean isThere = detect(rgbValues, lightingMargin, pixels);
      pixelCount = getBooleanPixelCount();
    }

    catch (Exception e) {
      e.printStackTrace();
    }

    return pixelCount;
  }

  //Blob Detection Pixel Method:
  public static ArrayList<Integer> detectBlobsPixelCount(int[][] rgbValues, int[] lightingMargin,
    int distanceThreshold) {
    //Main Blob Pixel Count (w/ Default):
    ArrayList<Integer> pixelCounts = new ArrayList<Integer>();

    try {
      //Gets the Blob Detection:
      int blobCount = detectBlobs(rgbValues, lightingMargin, distanceThreshold);
      pixelCounts = getBlobPixelCounts();
    }

    catch (Exception e) {
      e.printStackTrace();
    }

    //Returns the ArrayList:
    return pixelCounts;
  }

  //Nearest Pixel Method:
  public static int[] detectNearest(int rgbValues[][], int lightingMargin[]) {
    //Main Integer Coordinates:
    int coordinates[] = new int[2];

    try {
      //Gets the Coordinates:
      coordinates = getNearestPixel(rgbValues, lightingMargin);
    }

    catch (Exception e) {
      e.printStackTrace();
    }

    //Returns the Coordinates:
    return coordinates;
  }

  /* COORDINATE METHODS */

  //Get All Blob Coordinates Method:
  public static int[][] getCoordinates(int[][] rgbValues, int[] lightingMargin,
    int distanceThreshold) {
    //Main Coordinates Array (w/ Default):
    int coordinates[][] = null;

    try {
      //Gets the Blob Detection:
      int blobCount = detectBlobs(rgbValues, lightingMargin, distanceThreshold);
      ArrayList<Integer> x = getBlobX();
      ArrayList<Integer> y = getBlobY();
      coordinates = new int[2][blobCount];

      //Checks the Case:
      if (x.size() == y.size() && blobCount == x.size() && blobCount == y.size()) {
        //Loop Variable:
        int turns = 0;

        //Loops through Array:
        mainLoop: while (turns < x.size()) {
          //Sets the Coordinates:
          coordinates[0][turns] = x.get(turns);
          coordinates[1][turns] = y.get(turns);

          turns++;
        }
      }
    }

    catch (Exception e) {
      e.printStackTrace();
    }

    //Returns the Coordinates:
    return coordinates;
  }
}