/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.BlingInterface;
import frc.robot.subsystems.interfaces.DrivetrainInterface;

public class BlingA extends CommandBase {
    DrivetrainInterface drivetrain;
    BlingInterface bling;
    // Bling bling;
    private boolean done;
    int time;
  /**
   * Creates a new autoDriveForward.
   */
  public BlingA(DrivetrainInterface drivetrain, BlingInterface bling) {
    this.drivetrain = drivetrain;
    this.bling = bling;
    addRequirements((SubsystemBase)bling);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      time = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      // bling.setPatternRGBAll(69, 118, 42);
      if (time > 600) {
          done = true;
      } else {
          done = false;
      }
      time = time + 1;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return done;
    }
}
