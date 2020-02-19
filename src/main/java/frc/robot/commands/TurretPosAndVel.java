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
import frc.robot.subsystems.interfaces.TurretInterface;

public class TurretPosAndVel extends CommandBase {
  /**
   * Creates a new TurretIndex.
   */
  TurretInterface turret;
  double position;

  public TurretPosAndVel(TurretInterface turret_, double position_) {
    turret = turret_;
    position = position_;
    addRequirements((SubsystemBase) turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    SmartDashboard.putNumber("OLD Position", turret.getPosition());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.setPosition(position);
    SmartDashboard.putBoolean("Hit TPAV", true);
    SmartDashboard.putNumber("NEW Position", turret.getPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.setVelocity(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(turret.getPosition() >= position){
      SmartDashboard.putNumber("END position", position);
      return true;
    }
    else {
      return false;
    }
      
    }
  }

