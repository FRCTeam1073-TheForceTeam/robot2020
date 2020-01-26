/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.instances;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.ShooterInterface;

public class Shooter extends SubsystemBase implements ShooterInterface {
  private static WPI_TalonSRX shooterFlywheel1;
  private static WPI_TalonSRX shooterFlywheel2;

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    shooterFlywheel1 = new WPI_TalonSRX(22);
    shooterFlywheel2 = new WPI_TalonSRX(23);

    shooterFlywheel1.configFactoryDefault();
    shooterFlywheel2.configFactoryDefault();

    shooterFlywheel1.setSafetyEnabled(false);
    shooterFlywheel2.setSafetyEnabled(false);

    shooterFlywheel1.setNeutralMode(NeutralMode.Brake);
    shooterFlywheel2.setNeutralMode(NeutralMode.Brake);

    shooterFlywheel1.setInverted(false);
    shooterFlywheel2.setInverted(true);

    shooterFlywheel2.follow(shooterFlywheel1);

    shooterFlywheel1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    shooterFlywheel1.setSensorPhase(true);

    double P = 0;
    double I = 0;
    double D = 0;

    shooterFlywheel1.config_kP(0, P);
    shooterFlywheel1.config_kI(0, I);
    shooterFlywheel1.config_kD(0, D);

    shooterFlywheel1.setSelectedSensorPosition(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void setSpeed(double speed) {
    shooterFlywheel1.set(speed);
    shooterFlywheel2.set(speed);
  }

  @Override
  public void increaseSpeed(double speed) {

  }

  @Override
  public void decreaseSpeed(double speed) {
    
  }

  @Override
  public void setPID(double P, double I, double D){

  }

  @Override
  public double getSpeed() {
    return 0;
  }
}
