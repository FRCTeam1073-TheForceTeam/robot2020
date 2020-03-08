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
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.subsystems.interfaces.ControlPanelInterface;
import frc.robot.subsystems.interfaces.ShooterInterface;

enum DeadzoneRollerMode {
  DEADZONE_TRIGGER,
  CONTROL_PANEL
}

public class Shooter extends SubsystemBase implements ShooterInterface, ControlPanelInterface {
  private static WPI_TalonFX shooterFlywheel1;
  private static WPI_TalonFX shooterFlywheel2;
  private static CANSparkMax hood;
  private static CANDigitalInput hoodIndexer;
  private static CANEncoder hoodEncoder;
  private static CANEncoder hoodEncoder2;
  private static CANEncoder deadzoneRollerEncoder;
  private static CANPIDController hoodController;
  private static CANPIDController deadzoneRollerController;
  public CANSparkMax deadzoneRoller;

  private static final double flywheelTicksPerRevolution = 2048;
  private static final int hoodEncoderTPR = 1;//2048;
  private static final double minAngle = 19.64 * Math.PI / 180;
  private static final double maxAngle = 49.18 * Math.PI / 180;
  private static final double kMotorRadiansPerHoodRadian = 2.523808240890503 * 2 * Math.PI / (maxAngle - minAngle);

  DeadzoneRollerMode deadzoneRollerMode;
 
  
  /**
   * Creates a new Shooter.
   */

  public Shooter() {
    shooterFlywheel1 = new WPI_TalonFX(22);
    shooterFlywheel2 = new WPI_TalonFX(23);

    hood = new CANSparkMax(25, MotorType.kBrushless);
    hood.clearFaults();

    deadzoneRoller = new CANSparkMax(28, MotorType.kBrushless);

    shooterFlywheel1.configFactoryDefault();
    shooterFlywheel2.configFactoryDefault();

    hood.restoreFactoryDefaults();
    deadzoneRoller.restoreFactoryDefaults();
    deadzoneRollerEncoder = deadzoneRoller.getEncoder(EncoderType.kHallSensor, 1);
    deadzoneRollerController = deadzoneRoller.getPIDController();
    deadzoneRollerEncoder.setPosition(0);
    deadzoneRollerController.setIAccum(0);
//    deadzoneRollerController.setIMaxAccum(100, 0);
    deadzoneRollerController.setFeedbackDevice(deadzoneRollerEncoder);
    engageDeadzoneRoller();

    shooterFlywheel1.setSafetyEnabled(false);
    shooterFlywheel2.setSafetyEnabled(false);

    // shooterFlywheel1.setNeutralMode(NeutralMode.Brake);
    // shooterFlywheel2.setNeutralMode(NeutralMode.Brake);

    hood.setIdleMode(IdleMode.kBrake);

    // If it's not inverted, we're in for some problems.

    // shooterFlywheel1.setInverted(false);
    // shooterFlywheel2.setInverted(true);

    hood.setInverted(false);

    shooterFlywheel2.follow(shooterFlywheel1);
    shooterFlywheel2.setInverted(true);


    // shooterFlywheel1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    // shooterFlywheel1.setSensorPhase(true);

    double P = 1.5e-1;
    double I = 0;
    double D = 0;

    double hoodP = 2e-1;
    double hoodI = 4e-5;
    double hoodD = 0;

    double hoodP2 = 2e-2;
    double hoodI2 = 0;
    double hoodD2 = 0;

    shooterFlywheel1.config_kP(0, P);
    shooterFlywheel1.config_kI(0, I);
    shooterFlywheel1.config_kD(0, D);

    shooterFlywheel1.setSelectedSensorPosition(0);
    shooterFlywheel1.setIntegralAccumulator(0);
    shooterFlywheel1.configClosedloopRamp(0.25);

    // hoodIndexer = hood.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    hoodEncoder = hood.getEncoder(EncoderType.kHallSensor, hoodEncoderTPR);
    hoodEncoder.setPosition(0);
    hoodEncoder2 = hood.getAlternateEncoder(AlternateEncoderType.kQuadrature, 4096);
    hoodEncoder2.setPosition(0);
    hoodEncoder2.setInverted(true);
    hoodController = new CANPIDController(hood);
    hoodController.setFeedbackDevice(hoodEncoder);
    hoodController.setP(hoodP);
    hoodController.setI(hoodI);
    hoodController.setD(hoodD);
    hood.setClosedLoopRampRate(0.25);
  }
  
  public void setHoodPID(double P, double I, double D) {
    hoodController.setP(P);
    hoodController.setI(I);
    hoodController.setD(D);
  }

  private static boolean isHoodIndexed = false;
  private static boolean isHoodDisabled = false;
  private static double[] temperatures = new double[] { 15.6e6, -273.15 };
  private static double hoodAngle = 13.7e9;
  private static long lastTimestamp = 0;
  private static double flywheelVelocity = 0;
  private static final double hoodIndexAngle = 0;
  private static double hoodVelocityTarget = 0;
  private static double hoodInputPower = 0;

  @Override
  public void periodic() {
    if (OI.driverController.getStartButtonPressed()) {
      isHoodIndexed = !isHoodIndexed;
    }
    isHoodIndexed = (Math.abs(hoodInputPower) >= 0.015 && Math.abs(hoodEncoder.getVelocity()) <= 10);
    SmartDashboard.putBoolean("Hood Indexed?", hoodIsIndexed());
    hoodInputPower = hood.getAppliedOutput();
    temperatures[0] = shooterFlywheel1.getTemperature();
    temperatures[1] = shooterFlywheel2.getTemperature();
    hoodAngle = hoodEncoder.getPosition() * 2 * Math.PI;
    lastTimestamp = System.currentTimeMillis();
    flywheelVelocity = shooterFlywheel1.getSelectedSensorVelocity() * 2 * Math.PI * 10 / flywheelTicksPerRevolution;

    SmartDashboard.putNumber("Flywheel Error P", shooterFlywheel1.getClosedLoopError());
    SmartDashboard.putNumber("Flywheel Error P [RPM]",
        shooterFlywheel1.getClosedLoopError() * 600 / flywheelTicksPerRevolution);
    SmartDashboard.putNumber("Flywheel Error I", shooterFlywheel1.getIntegralAccumulator());
    SmartDashboard.putNumber("Flywheel Error D", shooterFlywheel1.getErrorDerivative());

    // SmartDashboard.putNumber("key", shooterFlywheel1.getMotorOutputPercent());
    // SmartDashboard.putNumber("OUT:", hood.getAppliedOutput());
    // SmartDashboard.putNumber("Velocity", hoodEncoder.getVelocity());
    SmartDashboard.putNumber("Position", hoodEncoder.getPosition());
    // SmartDashboard.putNumber("Position 2", hoodEncoder2.getPosition());
    // SmartDashboard.putNumber("Error", hoodEncoder.getPosition() - targetHoodAngle);

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
   * Sets the flywheel speed in radians/second. This sets a closed loop target
   * velocity for a flywheel and it will accelerate toward the target speed. The
   * speed will be clamped to the allowable speed limits.
   * 
   * @param speed in radians/second.
   */
  @Override
  public void setFlywheelSpeed(double speed) {
    shooterFlywheel1.set(ControlMode.Velocity, speed * flywheelTicksPerRevolution * 0.1 / (2 * Math.PI));
  }

  /**
   * Sets the flywheel power.
   */
  @Override
  public void setFlywheelPower(double power) {
    shooterFlywheel1.set(power);
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
    return 6400 * 2 * Math.PI / 60;
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

  double targetHoodAngle=0;

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
    if (false) {
      return false;
    }
    isHoodDisabled = false;
    double hoodAngle = kMotorRadiansPerHoodRadian * (angle - minAngle);
    hood.setIdleMode(IdleMode.kBrake);
    SmartDashboard.putNumber("angle", hoodAngle/(2*Math.PI));
    targetHoodAngle=hoodAngle/(2*Math.PI);
    hoodController.setReference(hoodAngle / (2 * Math.PI), ControlType.kPosition);
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
    hoodVelocityTarget = angle_rate / (2 * Math.PI) * 60;
    hoodController.setReference(hoodVelocityTarget, ControlType.kVelocity);
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
    double minAngle = 40 * Math.PI / 180;
    return (hoodAngle / (8 * Math.PI)) * (maxAngle - minAngle) + minAngle;
  }

  @Override
  public double getHoodVelocity() {
    return hoodEncoder.getVelocity();
  }

  /**
   * Return the minimum allowed hood angle in radians.
   * 
   * @return Hood angle in radians.
   */
  @Override
  public double getMinHoodAngle() {
    return minAngle;
  }

  /**
   * Return the maximum allowed hood angle in radians.
   * 
   * @return Hood angle in radians.
   */
  @Override
  public double getMaxHoodAngle() {
    return maxAngle;
  }

  @Override
  public boolean hoodIsIndexed() {
    return isHoodIndexed;
  }

  public void setHoodPower(double pow) {
    hood.set(pow);
  }

  private void setDeadzonerollerPID(double P, double I, double D) {
    
  }

  @Override
  public void engageControlPanel() {
    deadzoneRoller.restoreFactoryDefaults();
    setDeadzonerollerPID(0, 0, 0);
    deadzoneRollerMode = DeadzoneRollerMode.CONTROL_PANEL;
  }

  @Override
  public void setControlPanelVelocity(double velocity) {
    if (!(deadzoneRollerMode.equals(DeadzoneRollerMode.CONTROL_PANEL))) {
      return;
    }
    deadzoneRollerController.setReference(velocity / (2.0 * Math.PI), ControlType.kVelocity);
  }

  @Override
  public double getControlPanelVelocity() {
    return deadzoneRollerEncoder.getVelocity();
  }

  @Override
  public void engageDeadzoneRoller() {
    deadzoneRoller.restoreFactoryDefaults();
    deadzoneRoller.setInverted(true);
    setDeadzonerollerPID(1e-2, 0, 0);
    deadzoneRollerMode = DeadzoneRollerMode.DEADZONE_TRIGGER;
  }

  @Override
  public void setDeadzoneRollerVelocity(double velocity) {
    if (!(deadzoneRollerMode.equals(DeadzoneRollerMode.DEADZONE_TRIGGER))) {
      return;
    }
    deadzoneRollerController.setReference(velocity / (2.0 * Math.PI), ControlType.kVelocity);
  }

  @Override
  public double getDeadzoneRollerVelocity() {
    return deadzoneRollerEncoder.getVelocity();
  }
}