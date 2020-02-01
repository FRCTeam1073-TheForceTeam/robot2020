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
  private int updateCounter = 0;
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
    if (read(apiIndex(5,1), targetData) == true && targetData.length == 8) {
      targets[0].cx = (targetData.data[0] << 4 ) | ((targetData.data[1] & 0xf0) >> 4);
      targets[0].cy = ((targetData.data[1] & 0x0f) << 4) | targetData.data[2];
      targets[0].vx = targetData.data[3];
      targets[0].vy = targetData.data[4];
      targets[0].targetType = targetData.data[5];
      targets[0].quality = targetData.data[6];
      targets[0].skew = targetData.data[7];
      targets[0].timestamp = targetData.timestamp; // Assign the CANBus message timestamp
      lastUpdate = targetData.timestamp;
      updateCounter++;
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
      
      System.out.println(String.format("T: %d Cx: %d Cy: %d Vx: %d Vy: %d Type: %d Qual: %d Skew: %d", 
              lastUpdate, targets[0].cx, targets[0].cy, targets[0].vx, targets[0].vy, 
              targets[0].targetType, targets[0].quality, targets[0].skew));
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

}
