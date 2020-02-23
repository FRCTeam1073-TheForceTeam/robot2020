/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.TurretInterface;

public class WaitForTurret extends CommandBase {
  TurretInterface turret;
  double azimuth = 0.0;
  double tolerance = 0.01;

  public WaitForTurret(TurretInterface turret_, double azimuth_) {
    turret = turret_;
    azimuth = azimuth_;
    addRequirements((SubsystemBase) turret);
  }

  // Called instantly
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Waiting = ", true);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    turret.disable();
    SmartDashboard.putBoolean("Waiting = ", false);
  }

  @Override
  public boolean isFinished() {
    return turret.getPosition() == azimuth;
  }
}
