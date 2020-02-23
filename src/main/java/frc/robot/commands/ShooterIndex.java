/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.interfaces.ShooterInterface;

public class ShooterIndex extends CommandBase {
  /**
   * Creates a new ShooterIndex.
   */
  ShooterInterface shooter;
  double loops = 0;
  public ShooterIndex(ShooterInterface shooter_) {
    shooter = shooter_;
    addRequirements((SubsystemBase) shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    curr=System.currentTimeMillis();
    start = curr * 1;
  }

  boolean s = false;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    curr = System.currentTimeMillis();
    shooter.setHoodPower(-0.05 * OI.driverController.getRawAxis(1));
    SmartDashboard.putBoolean("signal", s = !s);
    SmartDashboard.putNumber("key", loops);
    SmartDashboard.putNumber("key3", (double) (curr-start));
    loops++;
  }

  long start = System.currentTimeMillis();
  long curr = System.currentTimeMillis();

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.resetHood();
    shooter.disableHood();
    SmartDashboard.putNumber("key2", 835);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;//(curr - start) > 200 && shooter.hoodIsIndexed();
  }
}
