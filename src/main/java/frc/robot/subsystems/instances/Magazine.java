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
  private static int cellCount;
  private static WPI_TalonSRX magMotor;
  private static final int ballDist = 2;

  public Magazine() {
    
    magMotor = new WPI_TalonSRX(24);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void run(double speed) {
    magMotor.set(speed);
  }

  @Override
  public int getCellCount() {
    return cellCount;
  }

  @Override
  public void addCell() {
    cellCount++;
  }

  @Override
  public void subtractCell() {
    cellCount--;
  }
}
