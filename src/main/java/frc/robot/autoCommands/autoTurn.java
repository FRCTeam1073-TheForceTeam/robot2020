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
import frc.robot.subsystems.interfaces.DrivetrainInterface;

public class autoTurn extends CommandBase {

  DrivetrainInterface drivetrain;
  private double maxVelocity;
  private double velocity;
  private double rotation;
  private double accelConstant = 0.0;
  private double initPose;
  private double currentPose;

  /**
   * Creates a new autoTurn.
   */
  public autoTurn(DrivetrainInterface drivetrain, double rotation, double maxVelocity) {

    this.drivetrain = drivetrain;
    this.rotation = rotation * (Math.PI / 180);
    this.maxVelocity = maxVelocity;
    addRequirements((SubsystemBase)drivetrain);
  }

  //  creates an autoTurn where you don't have to input the maximum speed
  public autoTurn(DrivetrainInterface drivetrain, double rotation) {

    this(drivetrain, rotation, Constants.MAX_DRIVETRAIN_VELOCITY);
  }

  // the following autoTurns are just set to certain angles
  public static autoTurn auto135left(DrivetrainInterface drivetrain) {
    return new autoTurn(drivetrain, -135);
  }
  
  public static autoTurn auto90left(DrivetrainInterface drivetrain) {
    return new autoTurn(drivetrain, -90);
  }

  public static autoTurn auto45left(DrivetrainInterface drivetrain) {
    return new autoTurn(drivetrain, -45);
  }

  public static autoTurn auto45right(DrivetrainInterface drivetrain) {
    return new autoTurn(drivetrain, 45);
  }
  
  public static autoTurn auto90right(DrivetrainInterface drivetrain) {
    return new autoTurn(drivetrain, 90);
  }
  
  public static autoTurn auto135right(DrivetrainInterface drivetrain) {
    return new autoTurn(drivetrain, 135);
  }
  
  public static autoTurn auto180(DrivetrainInterface drivetrain) {
    return new autoTurn(drivetrain, 180);
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initPose = drivetrain.getRobotPose().getRotation().getDegrees();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentPose = drivetrain.getRobotPose().getRotation().getDegrees();
    velocity = accelConstant * (rotation - (currentPose - initPose));

    // ensures it doesn't try to go faster than it's able to
    drivetrain.setVelocity( - Math.min(velocity, maxVelocity), java.lang.Math.min(velocity, maxVelocity));
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
    return ((currentPose - initPose) >= rotation);
  }
}
