/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.instances;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.TurretInterface;

public class Turret extends SubsystemBase implements TurretInterface {
  private WPI_TalonSRX turretRotator;
  private final double ticksPerRadian = 1440 / (2 * Math.PI); // TODO: get from CAD
  private double range = 3;          // radians
  private double indexOffset = -1.0; // radians  TODO: get from CAD
  private boolean velocityMode = true;
  private double velocityP = 0.25;
  private double velocityI = 0.01;
  private double velocityD = 0.0;

  private double positionP = 0.01;
  private double positionI = 0.001;
  private double positionD = 0.0;

  private boolean disabled = false;

  /* How to tell if the temperature sensor isn't working: if the robot
  thinks its temperature is 100 gigakelvin (the maximum temperature of
  a supernova) and the robot and surrounding continent haven't turned
  into a writhing vortex of quark-gluon plasma, it's probably just a bug.*/

  private double turretTemperature = 100e9;
  private double turretAngle = 0;
  private double turretVelocity = 0;
  private long timestamp = System.currentTimeMillis();
  private boolean leftLimitSwitch = false;
  private boolean rightLimitSwitch = false;
  private boolean indexed = false;
  
  public Turret() {
    turretRotator = new WPI_TalonSRX(24);
    turretRotator.configFactoryDefault();
    turretRotator.setSafetyEnabled(false); // TODO: fix motor safety
    turretRotator.setNeutralMode(NeutralMode.Brake);
    turretRotator.configPeakOutputForward(1);
    turretRotator.configPeakOutputReverse(-1);
    turretRotator.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    configVelocityMode();
    turretRotator.setSelectedSensorPosition(0);
    turretRotator.setIntegralAccumulator(0);
    if (turretRotator.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 30) != ErrorCode.OK) {
      System.out.println("ERROR! Forward Turret Limit Switch not configured");
    }
    if (turretRotator.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 30) != ErrorCode.OK) {
      System.out.println("ERROR! Reverse Turret Limit Switch not configured");
    }
    if (turretRotator.configClearPositionOnLimitR(true, 30) != ErrorCode.OK) {
      System.out.println("ERROR! Turret Postion limit is NOT set");
    } 
    SmartDashboard.putBoolean("Indexed = ", false);
  }
  //left = forward
  //right = back?
  //TODO: MUST resolve with real hardware
  @Override
  public void periodic() {
    turretAngle = (turretRotator.getSelectedSensorPosition() / ticksPerRadian) - indexOffset;
    turretVelocity = turretRotator.getSelectedSensorVelocity() * 0.1 / ticksPerRadian;
    turretTemperature = turretRotator.getTemperature();
    timestamp = System.currentTimeMillis();
    rightLimitSwitch = turretRotator.isRevLimitSwitchClosed() == 1;
  
    if (rightLimitSwitch == true) {
      SmartDashboard.putBoolean("Indexed = ", true);
      indexed = true;
    }
    SmartDashboard.putNumber("Turret Encoder", turretRotator.getSelectedSensorPosition());
    leftLimitSwitch = turretRotator.isFwdLimitSwitchClosed() == 1;

    // SmartDashboard.putNumber("Motor Output",turretRotator.getMotorOutputPercent());
    // SmartDashboard.putNumber("Velocity error (aka how much it's HOLDING ITSELF BACK)",turretRotator.getClosedLoopError());
    // SmartDashboard.putNumber("error", turretRotator.getClosedLoopError());
    // System.out.println("error" + turretRotator.getClosedLoopError());
    // This method will be called once per scheduler run
  }

  /**
   * Return the timestamp of the last turret state update.
   * @return the timestamp.
   */
  @Override
  public long getLastTurretUpdate() {
    return timestamp;
  }

  @Override
  public boolean setPosition(double azimuth) {
    if (!isIndexed()) {
      return false;
    }
    if (disabled) {
      disabled = false;
      turretRotator.setNeutralMode(NeutralMode.Coast);
    }
    if (velocityMode) {
      configPositionMode();
    }
    /**
     * Convert azimuth ang back into encoder ticks referencing the index offset 
     */
    // SmartDashboard.putBoolean("Actually Setting Position???", true);
    turretRotator.set(ControlMode.Position, ((azimuth + indexOffset) * ticksPerRadian)); 
    return true;
  }

  @Override
  public void deindex(){
    indexed = false;
  }

  /**
   * Return the maximum turret angle in radians.
   * our zero is the index offset (the limit switch)
   * @return Maximum turret angle in radians.
   */
  @Override
  public double getMaxPosition() {
    return indexOffset + range;
  }

  /**
   * Return the minimum turret angle in radians.
   * opposite of our index offset, assuming left = forward
   * @return Minimum turret angle in radians.
   */
  @Override
  public double getMinPosition() {
    return indexOffset;
  }

  /**
   * Set the turret rotation to a closed loop velocity given in radians/second.
   * This command works even if the turret has not been indexed.
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

    if (!velocityMode) {
      configVelocityMode();
    }

    // Multiplying speed by 10 because it's ticks/100ms, so 10*ticks/sec
    turretRotator.set(ControlMode.Velocity, angular_rate * ticksPerRadian * 10);
    return isIndexed();
  }

  /**
   * Disable the motor controls of the turret so that it will be 'limp'. Sending
   * any position or velocity command will re-enable the turret.
   */
  @Override
  public void disable() {
    disabled = true;
    turretRotator.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Return the azimuth angle of the turret in radians. This value is only
   * meaningful if isIndexed is true.
   * 
   * @return angle of the turret in Radians.
   */
  @Override
  public double getPosition() {
    return turretAngle;
  }

  /**
   * Return the current turret velocity in radians / second.
   * 
   * @return angular rate of the turret in radians/second.
   */
  @Override
  public double getVelocity() {
    return turretVelocity;
  }

  /**
   * Return true if the turret has been indexed.
   * 
   * @return True if the turret has been indexed, false if it has not been
   *         indexed.
   */
  @Override
  public boolean isIndexed() {
    return indexed;
  }

  @Override
  public void resetTurret() {
    turretRotator.setSelectedSensorPosition(0);
  }

  public void configVelocityMode() {
    turretRotator.config_kP(0, velocityP);
    turretRotator.config_kI(0, velocityI);
    turretRotator.config_kD(0, velocityD);
    velocityMode = true;
  }

  public void configPositionMode() {
    turretRotator.config_kP(0, positionP);
    turretRotator.config_kI(0, positionI);
    turretRotator.config_kD(0, positionD);
    velocityMode = false;
  }

}