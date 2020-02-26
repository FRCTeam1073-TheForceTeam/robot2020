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

public class BlingB extends CommandBase {
    BlingInterface bling;
    // Bling bling;
    private boolean done;
    int time;
  /**
   * Creates a new autoDriveForward.
   */
  public BlingB(BlingInterface bling) {
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
      bling.LEDRainbow();
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
    bling.setPatternRGBAll(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return done;
    }
}