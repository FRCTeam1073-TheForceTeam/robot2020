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
  private CANdata empty; 
  private CANdata data;

  /**
   * Creates a new OpenMV.
   */
  public OpenMVBase(int deviceId) {
    openmv = new CAN(deviceId, 173, 10);
    empty = new CANdata();
    data = new CANdata();
  }

  public void write(int APIIndex, byte[] data) {
    if(data.length <= 8)
      openmv.writePacket(data, 8);
  }

  public int apiIndex(int apiClass, int index){
      return (apiClass&0x03f<<4)|(index&0x0f);
    }

  public boolean read(int APIIndex, CANdata data) {
    return openmv.readPacketNew(APIIndex, data);
    }

  public void setMode(byte mode){
    byte[] message = new byte[1];
    message[0] = mode;
     write(apiIndex(1, 1), message);
  }

  public int[] readHeartbeat(){
    if(read(apiIndex(1, 2), data) == true){
      int[] heartbeat = new int[2];
      heartbeat[0] = data.data[0];
      heartbeat[1] = data.data[1] << 8 | data.data[2];
      return heartbeat;
    }
    return new int[0];
  }
  //creates a 
  public int[] readConfig(){
    if(read(apiIndex(1,0), data) == true){
      int[] config = new int[8];
      config[0] = data.data[0];//mode
      config[1] = data.data[1];
      config[2] = data.data[2];//simple target tracking
      config[3] = data.data[3];//Line Segment tracking
      config[4] = data.data[4];//color detection
      config[4] = data.data[5];//advanced target tracking
      config[4] = data.data[6];
      config[4] = data.data[7];
      return config;
    }
    return new int[0];
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(readHeartbeat().length != 0){
      System.out.println(readHeartbeat());
    }
    if(readConfig().length != 0){
      System.out.println(readConfig());
    }
  }
}
