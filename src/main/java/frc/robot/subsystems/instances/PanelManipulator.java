/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.instances;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.PanelManipulatorInterface;

public class PanelManipulator extends SubsystemBase implements PanelManipulatorInterface {

  private long lastUpdateTime;
  private double maxV;
  
  /**
   * Creates a new PanelManipulator.
   */

  public PanelManipulator() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void setVelocity(double velocity) {
    // TODO Auto-generated method stub

  }

  @Override
  public double getMaxVelocity() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public double getRotation() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public long getLastUpdateTime() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public void disable() {
    // TODO Auto-generated method stub

  }
}
