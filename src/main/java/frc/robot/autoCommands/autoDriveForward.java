/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autoCommands;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.interfaces.DrivetrainInterface;

public class autoDriveForward extends CommandBase {
  
  DrivetrainInterface drivetrain;
  private double maxVelocity;
  private double velocity;
  private double distance;
  private double accelConstant = 0;
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

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initPose = drivetrain.getRobotPose();
    SmartDashboard.putString("AUTO CMD: InitPose", initPose.toString());
  }

  int a = 0;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPose = drivetrain.getRobotPose();
    velocity = maxVelocity;
    // velocity = Math.min(Constants.MAX_DRIVETRAIN_VELOCITY,Math.max(-Constants.MAX_DRIVETRAIN_VELOCITY,(accelConstant * (distance - currentPose.minus(initPose).getTranslation().getNorm()))));
    SmartDashboard.putNumber("A", a++);
    // ensures that it doesn't try to go faster than it's able to

    SmartDashboard.putNumber("AUTO CMD: Auto Velocity", Math.min(velocity, maxVelocity));
    SmartDashboard.putString("AUTO CMD: currentPose",
      currentPose.getTranslation().getX()+","+currentPose.getTranslation().getY()
    );
    SmartDashboard.putString("AUTO CMD: initPose",
      initPose.getTranslation().getX()+","+initPose.getTranslation().getY()
    );
    SmartDashboard.putNumber("AUTO CMD: NORM",
      currentPose.minus(initPose).getTranslation().getNorm()
    );
    SmartDashboard.putNumber("AUTO CMD: DISTANCE", distance);

    //drivetrain.setPower(0.1, 0);
    drivetrain.setVelocity(velocity, 0.0);
  }
  boolean b=true;

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // stops the drivetrain
    drivetrain.setVelocity(0, 0);

    SmartDashboard.putBoolean("AutoDrive Status", interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putString("AUTO CMD: CurrentPose", currentPose.toString());
    SmartDashboard.putNumber("AUTO CMD: curr. init translation", currentPose.minus(initPose).getTranslation().getNorm());
    SmartDashboard.putBoolean("is: finished?",
        Math.hypot(
            currentPose.getTranslation().getX() - initPose.getTranslation().getX(),
            currentPose.getTranslation().getY() - initPose.getTranslation().getY()
        ) >= Math.abs(distance));

    return currentPose.minus(initPose).getTranslation().getNorm() >= Math.abs(distance);
  }
}