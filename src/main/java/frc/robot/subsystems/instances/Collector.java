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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.CollectorInterface;

public class Collector extends SubsystemBase implements CollectorInterface {
  boolean isLocked = false;
  private WPI_TalonSRX collectorMotor;
  private Solenoid collectorSolenoid;
  
  
  public Collector() {
    this.collectorSolenoid = new Solenoid(1);
    this.collectorMotor = new WPI_TalonSRX(24);
    this.collectorMotor.configFactoryDefault();
    this.collectorMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void run(double speed, String direction) {
    if (direction.equals("in")){
      speed *= 1.0;
    } else {
      speed *= -1.0;
    }
    if (direction.equals("in") && isLocked == true){
      collectorMotor.set(ControlMode.PercentOutput, 0.0);
    } else {
      collectorMotor.set(ControlMode.PercentOutput, speed);
    }

  }
  @Override
  public void collect(){
    run(1.0, "in");
  }
  @Override
  public void purge(){
    run(1.0, "out");
  }
  @Override
  public void raise(){
    collectorSolenoid.set(true);
  }
  @Override
  public void lower(){
    collectorSolenoid.set(false);
  }
  @Override
  public void stop(){
    run(0.0,"in");
  }
  @Override
  public void lockIntake(){
    isLocked = true;
  }
  @Override
  public void unlockIntake(){
    isLocked = false;
  }
}
