/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.ShooterInterface;

public class autoSetFlywheel extends CommandBase {

  ShooterInterface shooter;
  private double velocity;
  private double maxVelocity;

  /**
   * Creates a new autoSetFlywheel.
   */
  public autoSetFlywheel(ShooterInterface shooter, double velocity, double maxVelocity) {
    this.shooter = shooter;
    this.velocity = velocity;
    this.maxVelocity = maxVelocity;
    addRequirements((SubsystemBase)shooter);
  }

  // creates an autoSetFlywheel where you don't have to input the maximum speed
  public autoSetFlywheel(ShooterInterface shooter, double velocity) {
    this(shooter, velocity, shooter.getMaximumFlywheelSpeed());
  }

  // creates an autoSetFlywheel that sets the flywheel to its maximum speed
  public static autoSetFlywheel autoFlywheelMax(ShooterInterface shooter) {
    return new autoSetFlywheel(shooter, shooter.getMaximumFlywheelSpeed(), shooter.getMaximumFlywheelSpeed());
  }

  // creates an autoSetFlywheel that stops the flywheel
  public static autoSetFlywheel autoFlywheelOff(ShooterInterface shooter) {
    return new autoSetFlywheel(shooter, 0.0, shooter.getMaximumFlywheelSpeed());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // ensures that the flywheel doesn't try to go faster than it's able to
    shooter.setFlywheelSpeed(Math.min(velocity, maxVelocity));
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
