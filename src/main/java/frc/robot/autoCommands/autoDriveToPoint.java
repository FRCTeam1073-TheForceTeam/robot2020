/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.DrivetrainInterface;

public class autoDriveToPoint extends CommandBase {
  DrivetrainInterface drivetrain;
  /**
   * Creates a new autoDriveToPoint.
   */
  public autoDriveToPoint(double startY, double startX, double endY, double endX) {
    addRequirements((SubsystemBase)drivetrain);
    
    new autoTurn(drivetrain, returnAngleToTurn(startY, startX, endY, endX));
    new autoDriveForward(drivetrain, returnDistanceToTravel(startY, startX, endY, endX));
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

  public double returnAngleToTurn(double startY, double startX, double endY, double endX) {
    // Calculates the length of the legs of a triangle made with the robot path as the hypotenuse
    double XLength = Math.abs(endX - startX);
    double YLength = Math.abs(endY - startY);

    // Uses the inverse tangent to calculate the measure of the angle in degrees that the robot should turn
    double angleDegrees = Math.atan(YLength / XLength);

    // Allows the robot to move in the shortest way instead of all the way to the right
    if (angleDegrees > 180) {
      angleDegrees = -360 + angleDegrees;
    }

    // Converts degrees to radians
    double returnAngleRadians = Math.toRadians(angleDegrees);

    // Returns the angle needed to turn (in radians)
    return returnAngleRadians;
  }

  public double returnDistanceToTravel(double startY, double startX, double endY, double endX) {
    // Calculates the length of the legs of a triangle made with the robot path as the hypotenuse
    double XLength = Math.abs(endX - startX);
    double YLength = Math.abs(endY - startY);

    // Uses the pythagorean theorem to find the length of the hypotenuse
    double returnLength = Math.sqrt(Math.pow(XLength, 2) + Math.pow(YLength, 2));

    // Returns the length needed to travel (in meters)
    return returnLength;
  }
}
