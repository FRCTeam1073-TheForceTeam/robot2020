/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.subsystems.interfaces.LiftInterface;

public class LiftControls extends CommandBase {

  LiftInterface lift;

  /**
   * Creates a new LiftControls.
   */
  public LiftControls(LiftInterface lift_) {
    lift = lift_;
    addRequirements((SubsystemBase)lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (OI.operatorController.getYButton()) {
      lift.setLiftPower(.3);
    }
    if (OI.operatorController.getBButton()) {
      lift.setLiftPower(-.3);
    }
    if (OI.operatorController.getTriggerAxis(Hand.kRight) > .1) {
      lift.setHookPower(OI.operatorController.getTriggerAxis(Hand.kRight) * .75);
    }
    if (OI.operatorController.getTriggerAxis(Hand.kLeft) > .1) {
      lift.setHookPower(-OI.operatorController.getTriggerAxis(Hand.kLeft) * .75);
    }
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
