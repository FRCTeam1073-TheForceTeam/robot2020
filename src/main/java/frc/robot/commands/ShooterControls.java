/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.ShooterInterface;
import frc.robot.OI;

public class ShooterControls extends CommandBase {
  ShooterInterface shooter;
  double speed; 
  double flyVelocity = OI.operatorController.getRawAxis(1);
  double hoodVelocity = OI.operatorController.getRawAxis(5);

  public ShooterControls(ShooterInterface shooter_) {
    shooter = shooter_;
    addRequirements((SubsystemBase)shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // speed = 0;
    // shooter.setFlywheelSpeed(speed);
    shooter.setFlywheelSpeed(flyVelocity);
    shooter.setHoodVelocity(hoodVelocity);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
