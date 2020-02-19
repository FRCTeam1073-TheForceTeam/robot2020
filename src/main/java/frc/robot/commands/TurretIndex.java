/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.interfaces.TurretInterface;

public class TurretIndex extends CommandBase {
  /**
   * Creates a new TurretIndex.
   */
  TurretInterface turret;

  public TurretIndex(TurretInterface turret_) {
    turret = turret_;
    addRequirements((SubsystemBase) turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.turret.setVelocity(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.turret.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Robot.turret.isIndexed() == true) {
      return true;
    }
    else {
      return false;
    }
  }
}
