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
import frc.robot.OI;
import frc.robot.subsystems.interfaces.CollectorInterface;
import frc.robot.subsystems.interfaces.MagazineInterface;

public class MagazineControls extends CommandBase {
  MagazineInterface magazine;
  CollectorInterface collector;
  /**
   * Creates a new MagazineControls.
   */
  int cellCount;
  public MagazineControls(MagazineInterface magazine_, CollectorInterface collector_) {
    magazine = magazine_;
    collector = collector_;
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
    

    SmartDashboard.putNumber("Real Mag Power", magazine.getPower());
    SmartDashboard.putNumber("Mag Velocity", magazine.getVelocity());

    magazine.updateCellCount();
    cellCount = magazine.getCellCount();
    SmartDashboard.putNumber("Cell Count: ", cellCount);
    if(cellCount <= 0 || collector.getCollectorSolenoidIn() == false){
        magazine.setPower(0);
    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
