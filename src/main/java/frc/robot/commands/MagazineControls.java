/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.MagazineInterface;

public class MagazineControls extends CommandBase {
  MagazineInterface magazine;
  /**
   * Creates a new MagazineControls.
   */
  int cellCount;
  public MagazineControls(MagazineInterface magazine_) {
    magazine = magazine_;
    cellCount = 0;
    addRequirements((SubsystemBase)magazine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    magazine.updateCellCount();
    cellCount = magazine.getCellCount();
    SmartDashboard.putNumber("Cell Count: ", cellCount);
    SmartDashboard.putBoolean("Enterance: ", magazine.getEnteranceState());
    SmartDashboard.putBoolean("Exit: ", magazine.getExitState());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
