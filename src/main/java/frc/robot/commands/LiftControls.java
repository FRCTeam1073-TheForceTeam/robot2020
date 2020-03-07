/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.subsystems.instances.Drivetrain;
import frc.robot.subsystems.interfaces.LiftInterface;
import frc.robot.subsystems.interfaces.WinchInterface;

public class LiftControls extends CommandBase {

  LiftInterface lift;
  WinchInterface winch;

  /**
   * Creates a new LiftControls.
   */
  public LiftControls(LiftInterface lift_, WinchInterface winch_) {
    lift = lift_;
    winch = winch_;
    addRequirements((SubsystemBase)lift, (Drivetrain)winch);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Encoder Ticks", lift.getEncoderTicks());

    // Lift can only be run when drivetrain is engaged due to a mechanical limitation
    if (winch.isWinchEngaged() == false) {
      lift.liftExtend(OI.operatorController.getRawAxis(5) * (0.25));
    } else {
      lift.liftExtend(0);
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
