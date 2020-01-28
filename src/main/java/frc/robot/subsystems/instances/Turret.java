/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.instances;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.TurretInterface;

public class Turret extends SubsystemBase implements TurretInterface {
  WPI_TalonSRX turretRotator;
  private final double ticksPerRadian = 2048 / (2 * Math.PI);
  private double range = 350;
  private double P = 1;
  private double I = 0;
  private double D = 0;
  private boolean disabled = false;

  public Turret() {
    turretRotator = new WPI_TalonSRX(42);
    turretRotator.configFactoryDefault();
    turretRotator.setSafetyEnabled(false);
    turretRotator.setNeutralMode(NeutralMode.Brake);
    turretRotator.configPeakOutputForward(1);
    turretRotator.configPeakOutputReverse(-1);
    turretRotator.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    turretRotator.config_kP(0, P);
    turretRotator.config_kP(0, I);
    turretRotator.config_kP(0, D);
    turretRotator.setSelectedSensorPosition(0);
    turretRotator.setIntegralAccumulator(0);

  }

  @Override
  public void periodic() {
    turretRotator.set(0.1);
    // This method will be called once per scheduler run
  }

  @Override
  public boolean setPosition(double azimuth) {
    if (disabled) {
      disabled = false;
      turretRotator.setNeutralMode(NeutralMode.Brake);
    }
    turretRotator.set(ControlMode.Position, azimuth * ticksPerRadian);
    return true;
  }

  /**
   * Return the maximum turret angle in radians.
   * @return Maximum turret angle in radians.
   */
  @Override
  public double getMaxPosition() {
    return 0.5 * Units.degreesToRadians(range);
  }

  /**
   * Return the minimum turret angle in radians.
   * @return Minimum turret angle in radians.
   */
  @Override
  public double getMinPosition() {
    return -0.5 * Units.degreesToRadians(range);
  }

  /**
   * Set the turret rotation to a closed loop velocity given in radians/second. This command
   * works even if the turret has not been indexed.
   * 
   * @param angular_rate is rotation speed in radians/second.
   * @return True if turret is indexed, false if turret has not been indexed.
   */
  @Override
  public boolean setVelocity(double angular_rate) {
    if (disabled) {
      disabled = false;
      turretRotator.setNeutralMode(NeutralMode.Brake);
    }
    //Multiplying speed by 10 because it's ticks/100ms, so 10*ticks/sec
    turretRotator.set(ControlMode.Velocity, angular_rate * ticksPerRadian * 10);
    return false;
  }

  /**
   * Disable the motor controls of the turret so that it will be 'limp'. 
   * Sending any position or velocity command will re-enable the turret.
   */
  @Override
  public void disable() {
    disabled = false;
    turretRotator.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Return the azimuth angle of the turret in radians. This
   * value is only meaningful if isIndexed is true.
   * 
   * @return angle of the turret in Radians.
   */
  @Override
  public double getPosition() {
    return turretRotator.getSelectedSensorPosition()/ticksPerRadian;
  }

  /**
   * Return the current turret velocity in radians / second.
   * @return angular rate of the turret in radians/second.
   */
  @Override
  public double getVelocity() {
    return turretRotator.getSelectedSensorVelocity() * 0.1 / ticksPerRadian;
  }

  /**
   * Return true if the turret has been indexed.
   * @return True if the turret has been indexed, false if it has not been indexed.
   */
  @Override
  public boolean isIndexed() {
    return false;
  }
  
}
