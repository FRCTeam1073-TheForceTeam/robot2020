/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.TurretInterface;

public class TurretIndex extends CommandBase {

  TurretInterface turret;
  private double rotationSpeed;

  /**
   * Sets the rotation of the turret to 0
   */
  public TurretIndex(TurretInterface turret_, double rotationSpeed_) {
    rotationSpeed = rotationSpeed_;
    turret = turret_;
    addRequirements((SubsystemBase)turret);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.setVelocity(rotationSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turret.isIndexed();
  }
}
