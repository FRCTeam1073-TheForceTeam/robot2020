/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.instances;
import edu.wpi.first.hal.CANData;
import frc.robot.subsystems.interfaces.AdvancedTrackerInterface;

public class OMVPortTracker extends OpenMVBase implements AdvancedTrackerInterface {
  private CANData targetData;
  private AdvancedTrackerInterface.AdvancedTargetData [] targets;
  private long lastUpdate = 0;

  /**
   * Creates a new OMVPortTracker.
   */
  public OMVPortTracker(int deviceId) {
    super(deviceId);
    targetData = new CANData();
    targets = new AdvancedTrackerInterface.AdvancedTargetData[1]; // We only have 1 of these.
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
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Call base class:
    super.periodic();

    // Check for advanced target data:
    if (readAdvancedTracking()) {
      System.out.println("Advanced Tracking...");
      
      System.out.println(String.format("T: %d Cx: %d Cy: %d Type: %d Qual: %d Area: %f", 
              lastUpdate, targets[0].cx, targets[0].cy, 
              targets[0].targetType, targets[0].quality, targets[0].area));
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
   *
   */
   private void computeDistance(AdvancedTrackerInterface.AdvancedTargetData[] data){
     data[0].distance = (7.484 * 240) / (data.cy * 1.016);
   }
}