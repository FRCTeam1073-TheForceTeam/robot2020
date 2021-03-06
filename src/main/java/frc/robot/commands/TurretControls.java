/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.subsystems.interfaces.TurretInterface;

public class TurretControls extends CommandBase {

  TurretInterface turret;

  public TurretControls(TurretInterface turret_) {
    turret = turret_;
    addRequirements((SubsystemBase)turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double targetX = 10;
    // double targetY = 10;
    // Pose2d pose = Robot.drivetrain.getRobotPose();
    // double robotX=pose.getTranslation().getX();
    // double robotY=pose.getTranslation().getY();
    // double angle = Math.atan2(targetY - robotY, targetX - robotX);
    // double angle = OI.driverController.getRawAxis(0) * 10; 
    // double adjustedAngle = angle - Robot.drivetrain.getAngleRadians().getRadians();
  
  
    //Robot.turret.setPosition(angle);

    // System.out.println(Robot.turret.getPosition());
    double velocity = OI.operatorController.getRawAxis(0);
    turret.setVelocity(velocity);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double deadzone(double input){
    if(Math.abs(input) < 0.1) return 0.0;
    else return input;
  }
}
