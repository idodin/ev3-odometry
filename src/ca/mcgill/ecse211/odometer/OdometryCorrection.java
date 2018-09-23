/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import lejos.hardware.Sound;
import lejos.hardware.ev3.EV3;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;

public class OdometryCorrection implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  private Odometer odometer;
  private EV3ColorSensor color;
  private double[] currentPosition;
  private double[] lastPosition;
  private int yCount;
  private int xCount;
  private double correctedX, correctedY;
  private int lastColor = 2;
  private int currentColor;
  private static final double TILE_SIZE = 30.48;
  
  private static final Port cPort = LocalEV3.get().getPort("S1");

  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection() throws OdometerExceptions {

    this.odometer = Odometer.getOdometer();
    this.color = new EV3ColorSensor(cPort);

  }

  /**
   * Here is where the odometer correction code should be run.
   * 
   * @throws OdometerExceptions
   */
  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;

    while (true) {
      correctionStart = System.currentTimeMillis();
      
      currentColor = color.getColorID();
      if(currentColor - lastColor > 5) {
    	  
    	  currentPosition = odometer.getXYT();
    	  Sound.beep();
    	  
    	  yCount += Math.round(Math.cos(Math.toRadians(currentPosition[2])));
    	  xCount += Math.round(Math.sin(Math.toRadians(currentPosition[2])));
    	  
    	  //System.out.println("Theta: " + currentPosition[2]);
    	  //System.out.println("Y-count: " + yCount);
    	  //System.out.println("X-count: " + xCount);
    	  
    	  odometer.setXYT(Math.max(xCount-1,0)*TILE_SIZE,(Math.max(yCount-1,0))*TILE_SIZE,currentPosition[2]);
    	  
//    	  currentPosition = odometer.getXYT();
//    	  Sound.beep();
//    	  
//    	  try {
//    		  if(Math.abs(currentPosition[2]-lastPosition[2]) > 80.0) lastPosition = null; //If we have turned, reset.
//    		  
//    		  //Casting makes no sense as no further operations will be performed with these numbers. Just take the hit from the slower float operations.
//    		  correctedX = lastPosition[0] + TILE_SIZE * Math.sin(currentPosition[2]);
//        	  correctedY = lastPosition[1] + TILE_SIZE * Math.cos(currentPosition[2]);
//    	  } catch (NullPointerException e) {
//    		  correctedX = currentPosition[0];
//    		  correctedY = currentPosition[1];
//    	  } finally {
//    		  lastPosition = new double[] {correctedX, correctedY, currentPosition[2]};
//    	  }
//    	  
//    	  
//    	  odometer.setXYT(correctedX, correctedY, currentPosition[2]);
      }

      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here
        }
      }
    }
  }
}
