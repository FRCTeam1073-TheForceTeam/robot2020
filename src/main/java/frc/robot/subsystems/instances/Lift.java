/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.instances;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.LiftInterface;

public class Lift extends SubsystemBase implements LiftInterface {
  /**
   * Creates a new Lift.
   */
  private static Solenoid solenoid;


  public Lift() {
    
    solenoid = new Solenoid(17);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public void setBrakeOn() {

  }

  public void setBrakeOff() {
    
  }

  public boolean isBrakeSet() {
    return true;
    
  }

  public boolean isLiftFullyExtended() {
    return true;
  }

  public boolean isLiftFullyRetracted() {
    return true;
  }

  public double liftPosition() {
    return 0;
  }

  public void pinLift() {
    
  }

  public void unpinLift() {
    
  }

  public boolean isPinned() {
    return true;
  }
}
