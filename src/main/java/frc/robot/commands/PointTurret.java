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

public class PointTurret extends CommandBase {
  TurretInterface turret;
  double azimuth = 0.0;
  double tolerance = 0.0;

  public PointTurret(TurretInterface turret_, double azimuth_, double tolerance_) {
    turret = turret_;
    azimuth = azimuth_;
    tolerance = tolerance_;
    addRequirements((SubsystemBase) turret);
  }

  // Called instantly
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Going to azimuth of ", azimuth);
  }

  @Override
  public void execute() {
    turret.setPosition(azimuth);
  }

  @Override
  public void end(boolean interrupted){
  }

  @Override
  public boolean isFinished(){
    return turret.atPosition(azimuth, tolerance);
  }
// excecute() and end() are unneeded in instant commands
//isFinished() is always true in an instant command and SHOULDN'T be written out

}
