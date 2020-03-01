/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.interfaces.TurretInterface;

public class autoTurnTurret extends CommandBase {

  TurretInterface turret;
  private double rotation;
  private double velocity;
  private double maxVelocity;
  private double initTurretPosition;
  private double currentTurretPosition;
  private double accelConstant = 0.5;

  /**
   * Creates a new autoTurnTurret.
   */
  public autoTurnTurret(TurretInterface turret, double rotation, double maxVelocity) {
    this.turret = turret;
    this.rotation = rotation;
    this.maxVelocity = maxVelocity;
    addRequirements((SubsystemBase)turret);
  }

  // creates an autoTurnTurret where you don't have to input the maximum speed
  public autoTurnTurret(TurretInterface turret, double rotation) {
    this(turret, rotation, Constants.MAX_TURRET_VELOCITY);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initTurretPosition = turret.getPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentTurretPosition = turret.getPosition();
    velocity = accelConstant * (rotation - (currentTurretPosition - initTurretPosition));

    // ensures that it doesn't try to go faster than it's able to
    turret.setVelocity(Math.min(velocity, maxVelocity));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    // stops the turret
    turret.setVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((currentTurretPosition - initTurretPosition) >= rotation);
  }
}