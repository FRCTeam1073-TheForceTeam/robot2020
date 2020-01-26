/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.instances;
import edu.wpi.first.hal.CANData;
import frc.robot.subsystems.interfaces.PortTrackerInterface;

public class OMVPortTracker extends OpenMVBase implements PortTrackerInterface {
  private CANData targetData;
  private int quality = 0;
  private int cx = 0;
  private int cy = 0;
  private int vx = 0;
  private int vy = 0;
  private int ttype = 0;
  private int skew = 0;
  private int updateCounter = 0;
  private long lastUpdate = 0;


  /**
   * Creates a new OMVPortTracker.
   */
  public OMVPortTracker(int deviceId) {
    super(deviceId);
    targetData = new CANData();
  }

  // Updates our config and mode:
  public boolean readAdvancedTracking() {

    if (read(apiIndex(5,1), targetData) == true && targetData.length == 8) {
      cx = (targetData.data[0] << 4 ) | ((targetData.data[1] & 0xf0) >> 4);
      cy = ((targetData.data[1] & 0x0f) << 4) | targetData.data[2];
      vx = targetData.data[3];
      vy = targetData.data[4];
      ttype = targetData.data[5];
      quality = targetData.data[6];
      skew = targetData.data[7];
      updateCounter++;
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
      
      System.out.println(String.format("T: %d Cx: %d Cy: %d Vx: %d Vy: %d Type: %d Qual: %d Skew: %d", 
              lastUpdate, cx, cy, vx, vy, ttype, quality, skew));
    }

  }
}
