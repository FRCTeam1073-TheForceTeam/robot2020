/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.*;

public class ClosedLoopAiming extends CommandBase {

  TurretInterface turret;
  AdvancedTrackerInterface portTracker;
  ShooterInterface shooter;

  private double azimuthTarget;
  private int currX;
  private int currY;
  private boolean temporary;

  private final int targX;
  private final int targY;
  private final double accelConstant;

  /**
   * When activated, will align turret and shooter to the port vision target
   * @param turret_ Turret susbsytem on the robot
   * @param portTracker_ Vision tracker for the power port
   * @param shooter_ Shooter subsystem on the robot
   * @param temporary_ Set true to end the command when difference < sucess threshold
   */
  public ClosedLoopAiming(final TurretInterface turret_, final AdvancedTrackerInterface portTracker_, final ShooterInterface shooter_, boolean temporary_) {
    turret = turret_;
    portTracker = portTracker_;
    shooter = shooter_;
    addRequirements((SubsystemBase)turret, (SubsystemBase)portTracker);

    targX = 0;
    targY = 0;
    accelConstant = 1;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currX = portTracker.getAdvancedTargets()[0].cx;
    currY = portTracker.getAdvancedTargets()[0].cy;
    azimuthTarget = (targX - currX) * accelConstant;
    turret.setVelocity(azimuthTarget);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    turret.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(temporary) return targX - currX == 0;
    else return false;
  }
}
