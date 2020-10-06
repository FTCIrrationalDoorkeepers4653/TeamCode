package org.firstinspires.ftc.teamcode.Systems.Vision;

import android.graphics.Bitmap;
import java.util.ArrayList;
import lib.Analyze;
import lib.ImageRecognition;

public class Vision extends Analyze {
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

  /* RGB METHODS */

  //Gets the Array of RGB Values from an Image:
  public static int[][] getBitmapRGB(Bitmap image, int startX, int startY, int width, int height) {
    //Main RGB Array (w/ Default):
    int[][] rgbValues = new int[width][height];

    if (image != null && startX + width <= image.getWidth() && startY + height <= image.getHeight()) {
      //Gets RGB Values Array from Bitmap:
      int turnsWidth = startX;

      while (turnsWidth < startX + width) {
        //Loop Counter:
        int turnsHeight = startY;

        while (turnsHeight < startY + height) {
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

  /* MACHINE LEARNING METHODS */

  //Object Detection Method:
  public static boolean detectObject(int rgbValues[][], String identifier, int grid,
    double percentage) {
    //Gets the Object in RGB Values:
    boolean isThere = false;

    try {
      //Gets the Object Detection:
      isThere = ImageRecognition.authenticateImage(identifier, rgbValues,
        0, 0, percentage, grid);
    }

    catch (Exception e) {
      e.printStackTrace();
    }

    //Returns the Detection:
    return isThere;
  }

  //Object Training Method:
  public static void trainObject(int rgbValues[][], String identifier, int grid) {
    try {
      //Trains a Model on RGB Values:
      ImageRecognition.trainImage(identifier, rgbValues, 0, 0,
        0, grid);
    }

    catch (Exception e) {
      e.printStackTrace();
    }
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
}