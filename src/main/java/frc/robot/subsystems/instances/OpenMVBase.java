/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.instances;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.CAN;

public class OpenMVBase extends SubsystemBase {
  private CAN openmv;
  private CANData recvData;
  /// Mode of the OpenMV
  private int mode = 0;

  /// Heartbeat frame counter of the OpenMV
  private int frameCounter = 0;
  private int missedHeartbeats = 0;
  private long lastHeartbeat = 0; // Timestamp of last heartbeat.

  /// Configuration of the OpenMV
  private int simpleTargetTracking = 0;
  private int lineSegmentTracking = 0;
  private int colorDetection = 0;
  private int advancedTargetTracking = 0;

  /// Counter of configuration updates
  private int configurationUpdates = 0;


  /**
   * Creates a new OpenMV.
   */
  public OpenMVBase(int deviceId) {
    openmv = new CAN(deviceId, 173, 10);
    recvData = new CANData();
  }

  public void write(int APIIndex, byte[] data) {
    if(data.length <= 8)
      openmv.writePacket(data, 8);
  }

  public int apiIndex(int apiClass, int index){
      return ((apiClass & 0x03f) <<4) |(index & 0x0f);
    }

  public boolean read(int APIIndex, CANData data) {
    return openmv.readPacketNew(APIIndex, data);
    }

  public void setMode(byte mode){
    byte[] message = new byte[1];
    message[0] = mode;
     write(apiIndex(1, 1), message);
  }

  public int getMode() {
    return mode;
  }

  public int getFrameCounter() {
    return frameCounter;
  }

  public long getLastHeartbeat() {
    return lastHeartbeat;
  }

  public boolean readHeartbeat(){
    if(read(apiIndex(1, 2), recvData) == true && recvData.length == 3) {
      mode = recvData.data[0];
      frameCounter = (recvData.data[1] << 8) | recvData.data[2];
      lastHeartbeat = recvData.timestamp;
      return true;
    }
    return false;
  }

  // Updates our config and mode:
  public boolean readConfig(){
    if(read(apiIndex(1,0), recvData) == true && recvData.length == 8) {
      mode = recvData.data[0];//mode
      // config[1] = recvData.data[1];
      simpleTargetTracking = recvData.data[2];//simple target tracking
      lineSegmentTracking = recvData.data[3];//Line Segment tracking
      colorDetection = recvData.data[4];//color detection
      advancedTargetTracking = recvData.data[5];//advanced target tracking
      // config[4] = recvData.data[6];
      // config[4] = recvData.data[7];
      configurationUpdates++; // Count this configuration update.
      return true;
    }
    return false;
  }

  // Get config fields:
  public int getSimpleTargetTracking() {
    if (configurationUpdates > 0)
      return simpleTargetTracking;
    else
      return -1;
  }

  public int getLineSegmentTracking() {
    if (configurationUpdates > 0)
      return lineSegmentTracking;
    else
      return -1;
  }

  public int getColorDetection() {
    if (configurationUpdates > 0)
      return colorDetection;
    else
      return -1;
  }

  public int getAdvancedTargetTracking() {
    if (configurationUpdates > 0)
      return advancedTargetTracking;
    else
      return -1;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println("OMV Base...");
    // See if we got a heartbeat and update data.
    if (readHeartbeat()) {
      System.out.println("OpenMV Heartbeat");
      System.out.println(frameCounter);
      System.out.println(lastHeartbeat);
      missedHeartbeats = 0;
    } else {
      missedHeartbeats++;

      // If we miss too many:
      if (missedHeartbeats > 250) {
        System.out.println("OpenMV Missing heartbeat.");
        missedHeartbeats = 0;
      }

    }

    // See if we got a config message and update data
    if (readConfig()){
      System.out.println("OpenMV Configuration Received");
    }
  }
}
