/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autoCommands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.interfaces.DrivetrainInterface;

public class autoDriveForward extends CommandBase {
  
  DrivetrainInterface drivetrain;
  private double maxVelocity;
  private double velocity;
  private double distance;
  private double accelConstant = 0.0;
  Pose2d initPose;
  Pose2d currentPose;

  /**
   * Creates a new autoDriveForward.
   */
  public autoDriveForward(DrivetrainInterface drivetrain, double distance, double maxVelocity) {

    this.drivetrain = drivetrain;
    this.distance = distance;
    this.maxVelocity = maxVelocity;
    addRequirements((SubsystemBase)drivetrain);
  }

  // creates an autoDriveForward where you don't have to input the maxVelocity
  public autoDriveForward(DrivetrainInterface drivetrain, double distance) {

    this(drivetrain, distance, Constants.MAX_DRIVETRAIN_VELOCITY);
  }

  // creates an autoDriveForward where it only drives off of the initiation line
  public autoDriveForward autoInitLine(DrivetrainInterface drivetrain) {

    return new autoDriveForward(drivetrain, Constants.MIN_DISTANCE_INIT_LINE, Constants.MAX_DRIVETRAIN_VELOCITY);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    initPose = drivetrain.getRobotPose();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentPose = drivetrain.getRobotPose();
    velocity = accelConstant * (distance - currentPose.minus(initPose).getTranslation().getNorm());

    // ensures that it doesn't try to go faster than it's able to
    drivetrain.setVelocity(Math.min(velocity, maxVelocity), Math.min(velocity, maxVelocity));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // stops the drivetrain
    drivetrain.setVelocity(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return currentPose.minus(initPose).getTranslation().getNorm() >= distance;
  }
}
