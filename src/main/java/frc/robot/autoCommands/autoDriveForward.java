/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.DrivetrainInterface;

public class autoDriveForward extends CommandBase {
  DrivetrainInterface drivetrain;
  double maxVelocity = 100.0;
  double distance = 0.46;

  /**
   * Creates a new autoDriveForward.
   */
  public autoDriveForward(DrivetrainInterface drivetrain, double distance, final double maxVelocity) {
    this.drivetrain = drivetrain;
    this.distance = distance;
    this.maxVelocity = maxVelocity;
    addRequirements((SubsystemBase)drivetrain);
  }

  public autoDriveForward(DrivetrainInterface drivetrain, double distance) {
    this.drivetrain = drivetrain;
    this.distance = distance;
    maxVelocity = 100.0;
  }

  public autoDriveForward(DrivetrainInterface drivetrain) {
    this.drivetrain = drivetrain;
    distance = 0.46;
    maxVelocity = 100.0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
