/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autoCommands;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.DrivetrainInterface;

public class autoDriveToPoint extends SequentialCommandGroup {
  DrivetrainInterface drivetrain;

  /**
   * Creates a new autoDriveToPoint.
   */
  public autoDriveToPoint(double XLength, double YLength) {
    addRequirements((SubsystemBase)drivetrain);
    sequence(
      new autoTurn(drivetrain, returnAngleToTurn(XLength, YLength)),
      new autoDriveForward(drivetrain, returnDistanceToTravel(XLength, YLength))
    );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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

  public double returnAngleToTurn(double XLength, double YLength) {

    /* Uses the arctangent to calculate the measure of the angle in degrees that the robot should turn
    We used WPIlib's Rotation2d utility to subtract the target angle from the robot's angle the shortest way
    (if it's easier to get to the target angle by turning right, it will turn right, and vice versa with left) */
    Rotation2d angle = new Rotation2d(Math.atan2(YLength, XLength)).minus(drivetrain.getAngleRadians());


    // Returns the angle needed to turn (in radians)
    return angle.getRadians();
  }

  public double returnDistanceToTravel(double XLength, double YLength) {

    // Uses the pythagorean theorem to find the length of the hypotenuse
    double returnLength = Math.sqrt(Math.pow(XLength, 2) + Math.pow(YLength, 2));

    // Returns the length needed to travel (in meters)
    return returnLength;
  }
}