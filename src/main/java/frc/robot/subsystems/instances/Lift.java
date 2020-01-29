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

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.LiftInterface;

public class Lift extends SubsystemBase implements LiftInterface {
  /**
   * Creates a new Lift.
   */
  private static WPI_TalonSRX leftMotorLeader;
  private static WPI_TalonSRX rightMotorLeader;
  private static WPI_TalonSRX leftMotorFollower;
  private static WPI_TalonSRX rightMotorFollower;
  private static Compressor compressor;
  private static Solenoid solenoid;


  public Lift() {
    leftMotorLeader = new WPI_TalonSRX(12);
    rightMotorLeader = new WPI_TalonSRX(13);
    leftMotorFollower = new WPI_TalonSRX(14);
    rightMotorFollower = new WPI_TalonSRX(15);

    leftMotorLeader.configFactoryDefault();
    rightMotorLeader.configFactoryDefault();
    leftMotorFollower.configFactoryDefault();
    rightMotorFollower.configFactoryDefault();

    leftMotorLeader.setSafetyEnabled(true);
    rightMotorLeader.setSafetyEnabled(true);
    leftMotorFollower.setSafetyEnabled(true);
    rightMotorFollower.setSafetyEnabled(true);

    leftMotorLeader.setNeutralMode(NeutralMode.Brake);
    rightMotorLeader.setNeutralMode(NeutralMode.Brake);
    leftMotorFollower.setNeutralMode(NeutralMode.Brake);
    rightMotorFollower.setNeutralMode(NeutralMode.Brake);

    leftMotorLeader.configPeakOutputForward(1.0);
    rightMotorLeader.configPeakOutputReverse(-1.0);
    leftMotorFollower.configPeakOutputForward(1.0);
    rightMotorFollower.configPeakOutputReverse(-1.0);

    //leftMotorLeader.setInverted(true);

    leftMotorFollower.follow(leftMotorLeader);
    rightMotorFollower.follow(rightMotorLeader);

    compressor = new Compressor(16);
    compressor.setClosedLoopControl(true);
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

  public void engageWinch() {
    
  }

  public void disengageWinch() {
    
  }

  public boolean isWinchEngaged() {
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

  public void setPower(double power)  {
    
  }

  public void pinLift() {
    
  }

  public void unpinLift() {
    
  }

  public boolean isPinned() {
    return true;
  }
}
