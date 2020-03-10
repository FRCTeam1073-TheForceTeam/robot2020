/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.instances;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.TurretInterface;

/**
 * The turret subsystem uses the robot coordinate system. In the robot coordinate system
 * +X points out the front of the robot to the collector.
 * +Y points out the left side of the robot if you are sitting facing collector in robot.
 * +Z points upward from the floor.
 * The turret angle is a rotation about the Z axis defined using the right-hand rule.
 * 
 * The zero turret angle is with the turret facing directly forward.
 * 
 * Note that the physical limits of the turret limit positive turret rotation to a small amount.
 * Most of the range of the turret will be through negative turret angles (pointing backwards over
 * your right shoulder if you were sitting in the robot).
 * 
 * zero should be straight ahead.
 * -pi should be straight back.
 * -3pi/2 should be straight left.
 * -pi/2 should be straight right.
 * 
 */
public class Turret extends SubsystemBase implements TurretInterface {
  private TalonSRX turretRotator;
  private final double ticksPerRadian = 6441.318;
  /// This is the difference between max and min encoder angles in radians.
  /// Should be a positive number of radians > PI and < 2 PI.
  private double range = 4.0;          // radians  TODO: get from experiment
  /// This is the output angle when the encoder is zero ticks, in radians. Should be small positive number.
  private double indexOffset = 0.4;    // radians  TODO: get from measurement
  private boolean disabled = true;
  private boolean velocityMode = true;

  private double velocityP = 0.25;
  private double velocityI = 0.01;
  private double velocityD = 0.0;
  private double velocityFF = 0.4;

  private double positionP = 0.05;
  private double positionI = 0.001;
  private double positionD = 0.0;
  private double positionFF = 1.0;

  private double turretTemperature = -1.0;
  private double turretAngle = 0.0;
  private double turretVelocity = 0.0;
  private double turretCurrent = 0.0;
  private long timestamp = System.currentTimeMillis();
  private boolean forwardLimitSwitch = false;
  private boolean reverseLimitSwitch = false;
  private boolean indexed = false;
  
  public Turret() {
    turretRotator = new TalonSRX(24);
    if (turretRotator.configFactoryDefault(30) != ErrorCode.OK) {
      throw new RuntimeException("ERROR: Failed to configure turret default.");
    }
    // turretRotator.setSafetyEnabled(false);    // TODO: Revisit this for motor safety setup.
    turretRotator.setNeutralMode(NeutralMode.Coast);

    if (turretRotator.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30) != ErrorCode.OK) {
      throw new RuntimeException("ERROR! Failed to select turret feedback sensor.");
    }
    turretRotator.setSelectedSensorPosition(0);
    turretRotator.configClosedLoopPeriod(0, 5); // Only run as fast as we can get good velocity measurement.
    turretRotator.configVelocityMeasurementWindow(4);

    // Tighter deadband for neutral:
    turretRotator.configNeutralDeadband(0.01);

    // Maximum power levels
    turretRotator.configPeakOutputForward(1);
    turretRotator.configPeakOutputReverse(-1);
    // Nominal power levels (for stiction)
    turretRotator.configNominalOutputForward(0.0);
    turretRotator.configNominalOutputReverse(0.0);

    // Set current limits to 10 amps maximum.
    turretRotator.configContinuousCurrentLimit(10);

    turretRotator.selectProfileSlot(0, 0);
    turretRotator.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
    turretRotator.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30); // Debugging of motion magic trajectory
    configVelocityMode();

    // Trapezoidal profile and parameters:
    turretRotator.configMotionAcceleration(velocityToTicks(0.5) * 4, 30);
    turretRotator.configMotionCruiseVelocity(velocityToTicks(0.5), 30);
    turretRotator.configMotionSCurveStrength(0);

    if (turretRotator.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 30) != ErrorCode.OK) {
      throw new RuntimeException("ERROR! Forward Turret Limit Switch not configured.");
    }
    if (turretRotator.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 30) != ErrorCode.OK) {
      throw new RuntimeException("ERROR! Reverse Turret Limit Switch not configured.");
    }
    // Configure to zero encoder on positive limit position:
    if (turretRotator.configClearPositionOnLimitF(true, 30) != ErrorCode.OK) {
      throw new RuntimeException("ERROR! Turret Postion limit is NOT set.");
    } 
    SmartDashboard.putBoolean("Turret Indexed = ", false);
  }
  
  @Override
  public void periodic() {
    /// Sample States
    turretAngle = ticksToPosition(turretRotator.getSelectedSensorPosition());
    turretVelocity = ticksToVelocity(turretRotator.getSelectedSensorVelocity());
    turretCurrent = turretRotator.getStatorCurrent();
    turretTemperature = turretRotator.getTemperature();
    timestamp = System.currentTimeMillis();
    reverseLimitSwitch = turretRotator.isRevLimitSwitchClosed() == 1;
    forwardLimitSwitch = turretRotator.isFwdLimitSwitchClosed() == 1;

    /// Detect Indexing Event
    if (forwardLimitSwitch == true) {
      SmartDashboard.putBoolean("Turret Indexed = ", true);
      indexed = true;
    }

    SmartDashboard.putNumber("Turret Encoder", turretRotator.getSelectedSensorPosition());
    SmartDashboard.putNumber("Turret Angle", turretAngle);
    SmartDashboard.putNumber("Turret Velocity", turretVelocity);
    SmartDashboard.putNumber("Turret Output Power", turretRotator.getMotorOutputPercent());
    SmartDashboard.putNumber("Turrent Current", turretCurrent);

    if (turretRotator.getControlMode() == ControlMode.MotionMagic) {
      SmartDashboard.putNumber("Turret Target", turretRotator.getClosedLoopTarget(0));
      SmartDashboard.putNumber("Turret Traj Pos", turretRotator.getActiveTrajectoryPosition());
      SmartDashboard.putNumber("Turret Traj Vel", turretRotator.getActiveTrajectoryVelocity());
    } else {
      SmartDashboard.putNumber("Turret Target", 0.0);
      SmartDashboard.putNumber("Turret Traj Pos", 0.0);
      SmartDashboard.putNumber("Turret Traj Vel", 0.0);
    }

    SmartDashboard.putNumber("Turret Error P", turretRotator.getClosedLoopError());
    SmartDashboard.putNumber("Turret Error I", turretRotator.getErrorDerivative());
    SmartDashboard.putNumber("Turret Error D", turretRotator.getIntegralAccumulator());

    SmartDashboard.updateValues();
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

    // Enable if needed.
    if (disabled) {
      enable();
    }

    // Reconfigure mode if needed.
    if (velocityMode) {
      configPositionMode();
    }

    // Clamp to valid limits: Don't allow a command outside this range.
    if (azimuth > getMaxPosition())
      azimuth = getMaxPosition();

    if (azimuth < getMinPosition())
      azimuth = getMinPosition();
  
    // Set position for trajectory generator:
    System.out.println("Turret: setPosition() called ");
    System.out.println(positionToTicks(azimuth));
    turretRotator.set(ControlMode.MotionMagic, positionToTicks(azimuth));
    return true;
  }

  /**
   * Return the maximum turret angle in radians.
   * our encoder zero is the index offset (the forward switch),
   * which is the maximum angle physically possible.
   * 
   * @return Maximum turret angle in radians.
   */
  @Override
  public double getMaxPosition() {
    return indexOffset;
  }

  /**
   * Return the minimum turret angle in radians.
   * opposite of our index offset, moving in reverse from the positive angle.
   * 
   * @return Minimum turret angle in radians.
   */
  @Override
  public double getMinPosition() {
    return indexOffset - range;
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
      enable();
    }

    if (!velocityMode) {
      configVelocityMode();
    }

    turretRotator.set(ControlMode.Velocity, velocityToTicks(angular_rate));
    return isIndexed();
  }

  /**
   * Disable the motor controls of the turret so that it will be 'limp'. Sending
   * any position or velocity command will re-enable the turret.
   */
  @Override
  public void disable() {
    disabled = true;
    turretRotator.set(ControlMode.Disabled, 0);
    turretRotator.setNeutralMode(NeutralMode.Coast);
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
    indexed = false;
    disable();
    configVelocityMode();
  }

  private int velocityToTicks(double vel) {
    return (int)(vel * ticksPerRadian * 0.1);
  }
  
  private double ticksToVelocity(double ticks) {
    return ticks / ticksPerRadian * 10.0;
  }

  private int positionToTicks(double angle) {
    return (int)((angle - indexOffset) * ticksPerRadian);
  }

  private double ticksToPosition(double ticks) {
    return (ticks / ticksPerRadian) + indexOffset;
  }

  private void enable() {
    disabled = false;
    turretRotator.setNeutralMode(NeutralMode.Brake);
  }

  public void configVelocityMode() {
    turretRotator.config_kP(0, velocityP);
    turretRotator.config_kI(0, velocityI);
    turretRotator.config_kD(0, velocityD);
    turretRotator.config_kF(0, velocityFF);
    turretRotator.configMaxIntegralAccumulator(0, 500);
    turretRotator.setIntegralAccumulator(0);

    velocityMode = true;
  }

  public void configPositionMode() {
    turretRotator.config_kP(0, positionP);
    turretRotator.config_kI(0, positionI);
    turretRotator.config_kD(0, positionD);
    turretRotator.config_kF(0, positionFF);
    turretRotator.configMaxIntegralAccumulator(0, 500);
    turretRotator.setIntegralAccumulator(0);

    velocityMode = false;
  }

  @Override
  public boolean atPosition(double azimuth, double tolerance) {
    return Math.abs(azimuth - getPosition()) <= tolerance;
  }

}
