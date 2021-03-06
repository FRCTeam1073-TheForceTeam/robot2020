/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.instances;
import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.interfaces.AdvancedTrackerInterface;

public class OMVPortTracker extends OpenMVBase implements AdvancedTrackerInterface {
  private CANData targetData;
  private AdvancedTrackerInterface.AdvancedTargetData [] targets;
  private long lastUpdate = 0;
  private static final int centerX = 160; // Horizontal center of the image in pixels
  private static final int centerY = 120; // Vertical center of the image in pixels
  private static final double azimuthConv = 1.222/320; // Conversion ratio for the azimuth in degrees from pixels
  private static final double elevationConv = 0.977/240; // Conversion ratio for the elevation in degrees from pixels
  private static final double baseElev = 0.384; // Standard elevation of the camera in degrees
  private static final double portHeight = 2.49; // Height of the center of the inner port in meters
  private static final double trackerHeight = 0.8636; // Height of the port tracker camera in meters


  private static int loopIncrement = 0;

  /**
   * Creates a new OMVPortTracker.
   */
  public OMVPortTracker(int deviceId) {
    super(deviceId);
    targetData = new CANData();
    targets = new AdvancedTrackerInterface.AdvancedTargetData[1];   // We only have 1 of these right now.
    targets[0] = new AdvancedTrackerInterface.AdvancedTargetData();
    
  }

  // Updates our config and mode:
  public boolean readAdvancedTracking() {
    if (read(apiIndex(5, 1), targetData) == true && targetData.length == 8) {
      int cxhi = targetData.data[0] & 0xFF;
      int cxlo = targetData.data[1] & 0xF0;
      targets[0].cx = (cxhi << 4) | (cxlo >> 4);
      int cyhi = targetData.data[1] & 0x0F;
      int cylo = targetData.data[2] & 0xFF;
      targets[0].cy = (cyhi << 8) | cylo;
      int areahi = targetData.data[3] & 0xFF;
      int arealo = targetData.data[4] & 0xFF;
      targets[0].area = (areahi << 8) | arealo;
      targets[0].targetType = targetData.data[5];
      targets[0].quality = targetData.data[6];
      targets[0].timestamp = targetData.timestamp; // Assign the CANBus message timestamp
      lastUpdate = targetData.timestamp;
      return true;
    }
    else {
      if(loopIncrement++ >= 5){
        targets[0].quality = 0;
        loopIncrement = 0;
        return false;
      }
      else return true;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Call base class:
    super.periodic();

    // Check for advanced target data:
    if (readAdvancedTracking()) {
      computeAzimuth(targets[0]);
      computeElevation(targets[0]);
      computeDistance(targets[0]);
      computeRange(targets[0]);
      SmartDashboard.putNumber("tracking cx", targets[0].cx);
      SmartDashboard.putNumber("tracking cy", targets[0].cy);
      SmartDashboard.putNumber("tracking quality", targets[0].quality);
      SmartDashboard.putNumber("tracking azimuth", targets[0].azimuth);
      SmartDashboard.putNumber("tracking distance", targets[0].distance);
    }
  }

  @Override
  public AdvancedTrackerInterface.AdvancedTargetData[] getAdvancedTargets() {
    return targets;
  }

  @Override
  public long getLastAdvancedTargetUpdate() {
    return lastUpdate;
  }

  /**
   * Updates the distance variable in the target data instance variables in meters
   * @param data
   */
  private void computeDistance(AdvancedTrackerInterface.AdvancedTargetData data) {
    data.distance = (portHeight - trackerHeight) / Math.tan(data.elevation);
    //data.distance = (7.484 * 240) / (data.cy * 1.016);
  }

  /**
   * Updates the range variable in the target data instance variable in meters
   * @param data
   */
  private void computeRange(AdvancedTrackerInterface.AdvancedTargetData data) {
    data.range = (portHeight - trackerHeight) / Math.sin(data.elevation);
    //data.range = Math.sqrt(Math.pow(data.distance,2) - Math.pow(7.484, 2));
  }

  /**
   * Updates the azimuth variable in the target data instance variable in degrees
   * @param data
   */
  private void computeAzimuth(AdvancedTrackerInterface.AdvancedTargetData data) {
    if (data.quality == 0) data.azimuth = 0;
    else data.azimuth = -(centerX - data.cx) * azimuthConv;
  }

  /**
   * Updates the elevation variable in the target data instance variable in degrees
   * @param data
   */
  private void computeElevation(AdvancedTrackerInterface.AdvancedTargetData data) {
    if(data.quality == 0) data.elevation = 0;
    else data.elevation = (centerY - data.cy) * elevationConv + baseElev;
  }

}