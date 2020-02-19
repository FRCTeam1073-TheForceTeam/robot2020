/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//TODO: write in the deadzone roller

package frc.robot.subsystems.instances;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.ShooterInterface;

public class Shooter extends SubsystemBase implements ShooterInterface {
  private static WPI_TalonFX shooterFlywheel1;
  private static WPI_TalonFX shooterFlywheel2;
  private static CANSparkMax hood;
  private static CANDigitalInput hoodIndexer;
  private static CANEncoder hoodEncoder;
  private static CANPIDController hoodController;
  private static final double flywheelTicksPerRevolution = 2048;
  private static final int hoodEncoderTPR = 2048;
  // private static CANSparkMax deadzone;
  // private static CANPIDController deadzoneController;
  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    shooterFlywheel1 = new WPI_TalonFX(22);
    shooterFlywheel2 = new WPI_TalonFX(23);
    hood = new CANSparkMax(24, MotorType.kBrushless);

    shooterFlywheel1.configFactoryDefault();
    shooterFlywheel2.configFactoryDefault();
    hood.restoreFactoryDefaults();

    shooterFlywheel1.setSafetyEnabled(false);
    shooterFlywheel2.setSafetyEnabled(false);

    shooterFlywheel1.setNeutralMode(NeutralMode.Brake);
    shooterFlywheel2.setNeutralMode(NeutralMode.Brake);
    hood.setIdleMode(IdleMode.kBrake);

    // If it's not inverted, we're in for some problems.
    shooterFlywheel1.setInverted(false);
    shooterFlywheel2.setInverted(true);

    hood.setInverted(false);

    shooterFlywheel2.follow(shooterFlywheel1);

    shooterFlywheel1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    shooterFlywheel1.setSensorPhase(true);

    double P = 0;
    double I = 0;
    double D = 0;

    double hoodP = 0;
    double hoodI = 0;
    double hoodD = 0;

    shooterFlywheel1.config_kP(0, P);
    shooterFlywheel1.config_kI(0, I);
    shooterFlywheel1.config_kD(0, D);

    shooterFlywheel1.setSelectedSensorPosition(0);
    hoodIndexer = hood.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    hoodEncoder = hood.getEncoder(EncoderType.kQuadrature, hoodEncoderTPR);
    hoodController = new CANPIDController(hood);
    hoodController.setFeedbackDevice(hoodEncoder);
    hoodController.setP(hoodP);
    hoodController.setI(hoodI);
    hoodController.setD(hoodD);
  }

  private static boolean isHoodIndexed = false;
  private static boolean isHoodDisabled = false;
  private static double[] temperatures = new double[] { 15.6e6, -273.15 };
  private static double hoodAngle = 0;
  private static long lastTimestamp = 0;
  private static double flywheelVelocity = 0;
  private static final double hoodIndexAngle = 0;

  @Override
  public void periodic() {
    isHoodIndexed = hoodIndexer.get();
    temperatures[0] = shooterFlywheel1.getTemperature();
    temperatures[1] = shooterFlywheel2.getTemperature();
    hoodAngle = hoodEncoder.getPosition() * 2 * Math.PI;
    lastTimestamp = System.currentTimeMillis();
    flywheelVelocity = shooterFlywheel1.getSelectedSensorVelocity() * 10 / flywheelTicksPerRevolution;
  }

  /**
   * Return the timestamp of the last hood update.
   * 
   * @return
   */
  @Override
  public long getLastShooterUpdate() {
    return lastTimestamp;
  }

  /**
   * Set the flywheel speed in radians/second. This sets a closed loop target
   * velocity for a flywheel and it will accelerate toward the target speed. The
   * speed will be clamped to the allowable speed limits.
   * 
   * @param speed in radians/second.
   */
  @Override
  public void setFlywheelSpeed(double speed) {
    shooterFlywheel1.set(ControlMode.Velocity, speed * 10 / flywheelTicksPerRevolution);
  }

  /**
   * Return the maximum flywheel speed.
   * 
   * @return Maximum flywheel speed in radians/second.
   */
  @Override
  public double getMaximumFlywheelSpeed() {
    // Value needs to be determined experimentally
    // TODO: Test this.
    return 600.0;
  }

  /**
   * Disable the motor axis of the flywheel so that it is limp. Sending a new
   * speed command will re-enable the flywheel.
   */
  @Override
  public void disableFlywheel() {
    shooterFlywheel1.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Return the flywheel speed in radians/second.
   * 
   * @return Flywheel speed in radians/second.
   */
  @Override
  public double getFlywheelSpeed() {
    return flywheelVelocity;
  }

  /**
   * Return the flywheel motor internal temperature.
   * 
   * @return temperature in degrees C.
   */
  @Override
  public double[] getInternalTemperature() {
    return temperatures;
  }

  /**
   * Resets hood.
   */
  public void resetHood() {
    hoodEncoder.setPosition(hoodIndexAngle);
  }

  /**
   * Set the target hood angle. The hood will move toward this angle and hold this
   * angle under closed loop control. This command is only valid if the hood has
   * been indexed.
   *
   * @param angle Angular postiion of the hood in radians.
   * @return True if the hood is indexed and we can set angles. False if the hood
   *         is not indexed.
   */
  @Override
  public boolean setHoodAngle(double angle) {
    if (isHoodIndexed) {
      return false;

    }
    isHoodDisabled = false;
    hood.setIdleMode(IdleMode.kBrake);
    hoodController.setReference(angle / (2 * Math.PI), ControlType.kPosition);
    return true;
  }

  /**
   * Set the hood to move in velocity mode which can be done without indexing the
   * hood.
   * 
   * @param angle_rate in radians/sec
   * @return True if the hood is indexed,
   */
  @Override
  public boolean setHoodVelocity(double angle_rate) {
    if (isHoodDisabled) {
      return false;

    }
    hoodController.setReference(angle_rate / (2 * Math.PI) * 60, ControlType.kVelocity);
    return hoodIsIndexed();
  }

  /**
   * Disable the hood axis control so that it is "limp". Setting a new hood angle
   * will re-enable the hood axis control.
   */
  @Override
  public void disableHood() {
    hood.setIdleMode(IdleMode.kBrake);
    isHoodDisabled = true;
  }

  /**
   * Return the current hood angle in radians.
   */
  @Override
  public double getHoodAngle() {
    return hoodAngle;
  }

  @Override
  public double getHoodVelocity() {
    return 0;
  }

  /**
   * Return the minimum allowed hood angle in radians.
   * 
   * @return Hood angle in radians.
   */
  @Override
  public double getMinHoodAngle() {
    return 0.0;
  }

  /**
   * Return the maximum allowed hood angle in radians.
   * 
   * @return Hood angle in radians.
   */
  @Override
  public double getMaxHoodAngle() {
    return Math.PI;
  }

  @Override
  public boolean hoodIsIndexed() {
    return isHoodIndexed;
  }
}
