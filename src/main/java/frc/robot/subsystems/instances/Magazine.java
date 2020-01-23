/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.instances;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.MagazineInterface;

public class Magazine extends SubsystemBase implements MagazineInterface {
  /** 
   * Creates a new Magazine.
   */
  public Magazine() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void run(double speed) {
    // TODO Auto-generated method stub

  }

  @Override
  public int getCellCount() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public void addCell() {
    // TODO Auto-generated method stub

  }

  @Override
  public void subtractCell() {
    // TODO Auto-generated method stub

  }
}
