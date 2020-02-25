/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.TurretInterface;

public class autoTurnTurret extends CommandBase {

  TurretInterface turret;
  private double rotation;
  private double maxVelocity;
  private double initTurretPosition;

  /**
   * Creates a new autoTurnTurret.
   */
  public autoTurnTurret(TurretInterface turret, double rotation, double maxVelocity) {
    this.turret = turret;
    this.rotation = rotation;
    addRequirements((SubsystemBase)turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initTurretPosition = (turret.getPosition() * (1 / Math.PI) * 180);
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
