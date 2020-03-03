/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.subsystems.interfaces.CollectorInterface;
import static frc.robot.subsystems.interfaces.CollectorInterface.CollectorDirection;

public class CollectorControls extends CommandBase {
  CollectorInterface collect;
  /**
   * Creates a new CollectControls.
   */
  public CollectorControls(CollectorInterface collect_) {
    collect = collect_;
    addRequirements((SubsystemBase)collect);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
  }

  boolean isRaised = true;
  // Called every time the scheduler runs while the command is scheduled.
  double pow = 1;
  @Override
  public void execute() {
    if (OI.driverController.getBumper(Hand.kRight)) {
      collect.lower();
      isRaised = false;
    } else {
      collect.raise();
      isRaised = true;
    }

    // if (OI.driverController.getAButtonPressed()){
    //   collect.raise();
    //   isRaised = true;
    // } else if (OI.driverController.getBButtonPressed()) {
    //   collect.lower();
    //   isRaised = false;
    // }

    if (isRaised == false) {
      collect.collect();
    } else {
      collect.stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
