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
import frc.robot.subsystems.interfaces.ShooterInterface;

public class autoSetHood extends CommandBase {

  ShooterInterface shooter;
  private double rotation;
  private double velocity;
  private double maxVelocity;
  private double initHoodPosition;
  private double currentHoodPosition;
  private double accelConstant;

  /**
   * Creates a new autoSetHood.
   */
  public autoSetHood(ShooterInterface shooter, double rotation, double maxVelocity) {
    this.shooter = shooter;
    this.rotation = rotation;
    this.maxVelocity = maxVelocity;
    addRequirements((SubsystemBase)shooter);
  }

  // creates an autoSetHood where you don't have to input the maximum speed
  public autoSetHood(ShooterInterface shooter, double rotation) {
    this(shooter, rotation, Constants.MAX_HOOD_VELOCITY);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initHoodPosition = (shooter.getHoodAngle() * (1 / Math.PI) * 180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentHoodPosition = (shooter.getHoodAngle() * (1 / Math.PI) * 180);
    velocity = accelConstant * (rotation - (currentHoodPosition - initHoodPosition));

    // secures that it doesn't try to go faster than it's able to
    if (velocity <= maxVelocity){
    shooter.setHoodVelocity(velocity);
    }
    else {
      shooter.setHoodVelocity(maxVelocity);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // stops the hood
    shooter.setHoodVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((currentHoodPosition - initHoodPosition) >= rotation);
  }
}
