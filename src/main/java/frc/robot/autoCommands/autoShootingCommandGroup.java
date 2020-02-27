/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.interfaces.DrivetrainInterface;
import frc.robot.subsystems.interfaces.ShooterInterface;
import frc.robot.subsystems.interfaces.TurretInterface;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class autoShootingCommandGroup extends SequentialCommandGroup {

  DrivetrainInterface drivetrain;
  ShooterInterface shooter;
  TurretInterface turret;

  /**
   * Creates a new autoShootingCommandGroup.
   */
  public autoShootingCommandGroup(double drivetrainRotation, double drivetrainDistance, double turretRotation,
      double hoodRotation, double flywheelVelocity) {
    super();

    addCommands(
      new autoTurn(drivetrain, drivetrainRotation),

      new autoDriveForward(drivetrain, drivetrainDistance)
      .alongWith(
        new autoTurnTurret(turret, turretRotation),
        new autoSetHood(shooter, hoodRotation)
      ),

      new autoSetFlywheel(shooter, flywheelVelocity)
    );
  }
}