/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.instances;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.CollectorInterface;

public class Collector extends SubsystemBase implements CollectorInterface {
  boolean isLocked = false;
  private WPI_TalonSRX collectorMotor;
  private Solenoid collectorSolenoidOut, collectorSolenoidIn;
  
  lastTimestamp = System.currentTimeMillis();
  
  public Collector() {
    this.collectorSolenoidOut = new Solenoid(1, 6);
    this.collectorSolenoidIn = new Solenoid(1, 0);
    this.collectorMotor = new WPI_TalonSRX(27);
    this.collectorMotor.configFactoryDefault();
    collectorMotor.configContinuousCurrentLimit(10);
    this.collectorMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putBoolean("CollectorOut", collectorSolenoidOut.get());
    // SmartDashboard.putBoolean("CollectorIn", collectorSolenoidIn.get());
    // This method will be called once per scheduler run
  }

  @Override
  public void run(double speed, CollectorDirection direction) {
    if (direction==CollectorDirection.IN){
      speed *= 1.0;
    } else {
      speed *= -1.0;
    }
    if (direction==CollectorDirection.IN && isLocked == true){
      collectorMotor.set(ControlMode.PercentOutput, 0.0);
    } else {
      collectorMotor.set(ControlMode.PercentOutput, speed);
    }

  }

  
  @Override
  public void collect(){
    run(0.7, CollectorDirection.IN);
  }
  @Override
  public void purge(){
    run(0.7, CollectorDirection.OUT);
  }
  @Override
  public void raise(){
    collectorSolenoidOut.set(true);
    collectorSolenoidIn.set(false);
    SmartDashboard.putBoolean("CollectorRaise", true);
  }
  @Override
  public void lower(){
    collectorSolenoidOut.set(false);
    collectorSolenoidIn.set(true);
    SmartDashboard.putBoolean("CollectorRaise", false);
  }
  @Override
  public void stop(){
    run(0.0, CollectorDirection.OUT);
  }
  @Override
  public void lockIntake(){
    isLocked = true;
  }
  @Override
  public void unlockIntake(){
    isLocked = false;
  }

  public void shut() {
    collectorSolenoidIn.set(false);
    collectorSolenoidOut.set(false);
  }

  @Override
  public boolean getCollectorSolenoidIn(){
    return collectorSolenoidIn.get();
  }
}
