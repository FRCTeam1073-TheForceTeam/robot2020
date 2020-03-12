/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.InterpolatorTable;
import frc.robot.components.InterpolatorTable.InterpolatorTableEntry;
import frc.robot.subsystems.interfaces.DrivetrainInterface;
import frc.robot.subsystems.interfaces.ShooterInterface;
import frc.robot.subsystems.interfaces.TurretInterface;

public class ExperimentalInterpolatorAiming extends CommandBase {

  /**
   * Table to convert distance to hood angle in rotations.
   */
  InterpolatorTable hoodAngleTable;
  /**
   * Table to convert distance to flywheel speed in (fake) RPM.
   */
  InterpolatorTable shooterVelocityTable;
  
  /**
   * Flywheel PID is undertuned, but changing it invalidates
   * our data, so this conversion constant should 
   * the effect well enough.
   */
  public final double flywheelRPMConversionFactor = 1.351;


  double distance;
  double baseDistance = 0;
  TurretInterface turret;
  ShooterInterface shooter;
  DrivetrainInterface drivetrain;
  /**
   * Creates a new ExperimentalInterpolatorAiming.
   */
  public ExperimentalInterpolatorAiming(TurretInterface turret_, ShooterInterface shooter_, DrivetrainInterface drivetrain_, double distance_) {
    baseDistance = distance_;
    distance = 0;
    turret = turret_;
    shooter = shooter_;
    drivetrain = drivetrain_;
    addRequirements((SubsystemBase) turret, (SubsystemBase) shooter);
    hoodAngleTable = new InterpolatorTable(
      new InterpolatorTableEntry(Units.feetToMeters(4.0), 0.0),
      new InterpolatorTableEntry(Units.feetToMeters(6.0), 0.0),
      new InterpolatorTableEntry(Units.feetToMeters(8.0), 0.28),
      new InterpolatorTableEntry(Units.feetToMeters(10.0),0.7696),
      new InterpolatorTableEntry(Units.feetToMeters(12.0),1.259),
      new InterpolatorTableEntry(Units.feetToMeters(14.0),1.014),
      new InterpolatorTableEntry(Units.feetToMeters(16.0),1.504),
      new InterpolatorTableEntry(Units.feetToMeters(18.0),1.749),
      new InterpolatorTableEntry(Units.feetToMeters(20.0),1.749)
    );
    shooterVelocityTable = new InterpolatorTable(
      new InterpolatorTableEntry(Units.feetToMeters(4.0), 2781),
      new InterpolatorTableEntry(Units.feetToMeters(6.0), 2820),
      new InterpolatorTableEntry(Units.feetToMeters(8.0), 3022),
      new InterpolatorTableEntry(Units.feetToMeters(10.0),3195),
      new InterpolatorTableEntry(Units.feetToMeters(12.0),3597),
      new InterpolatorTableEntry(Units.feetToMeters(14.0),3637),
      new InterpolatorTableEntry(Units.feetToMeters(16.0),4070),
      new InterpolatorTableEntry(Units.feetToMeters(18.0),4288),
      new InterpolatorTableEntry(Units.feetToMeters(20.0),4436)
    );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetRobotOdometry();
  }


  double speed = 0;
  double angle = 0;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distance = baseDistance + drivetrain.getRobotPose().getTranslation().getX();
    angle = hoodAngleTable.getValue(distance);
    speed = shooterVelocityTable.getValue(distance);
    SmartDashboard.putString("ExperimentalInterpolatorAiming: ", "ANGLE=" + angle + ", SPEED=" + speed);
    shooter.setRawHoodAngle(angle);
    shooter.setFlywheelSpeed(flywheelRPMConversionFactor * speed * Math.PI / 30.0);
    SmartDashboard.putNumber("Flywheel Speed 1", speed * Math.PI / 30.0);
    SmartDashboard.putNumber("Flywheel Speed 2", shooter.getFlywheelSpeed());
    if (Math.abs(
      (-speed * Math.PI / 30.0) - shooter.getFlywheelSpeed())<20) {
      shooter.setDeadzoneRollerPower(1);
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
