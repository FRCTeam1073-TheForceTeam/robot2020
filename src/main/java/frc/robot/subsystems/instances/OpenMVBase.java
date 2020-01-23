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
  private CANData data;

  /**
   * Creates a new OpenMV.
   */
  public OpenMVBase(int deviceId, int deviceType) {
    openmv = new CAN(deviceId, 170, deviceType);
    data = new CANData();
  }

  public void write(int APIIndex, byte[] data) {
    Integer len = data.length;
    if(len <= 8)
      openmv.writePacket(data, 8);
  }

  public String read() {
    boolean canCam = openmv.readPacketNew(4, data);
    if(canCam == true) {
      String outstr = new String(data.data);
      return outstr;
    }
    return "";
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
