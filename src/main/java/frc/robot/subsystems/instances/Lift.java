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
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.LiftInterface;

public class Lift extends SubsystemBase implements LiftInterface {
  /**
   * Creates a new Lift.
   */
  Solenoid solenoid = new Solenoid(7);

  WPI_TalonSRX liftWinch = new WPI_TalonSRX(24);

  WPI_VictorSPX hookWinch = new WPI_VictorSPX(22);
  WPI_VictorSPX hookWinch2 = new WPI_VictorSPX(23);

  // that sets minimum possible value the potentiometer can return (when the
  // voltage from the potentiometer is 0 (the minimum))
  private final double minLiftExtension = 0.0;
  // that sets maximum possible value the potentiometer can return (when the
  // voltage from the potentiometer is 1 (the maximum))
  private final double maxLiftExtension = 1.0;

  // these values have to be set to the measured physical limits using the
  // potentiometer HAVE TO BE SET
  private final double physicalMinExtension = 0.0;
  private final double physicalMaxExtension = 10.0;

  // defines the potentiometerPort HAS TO BE SET
  private final int potentiometerPort = 0;

  // Initializes an AnalogInput on port "potentiometerPort"
  AnalogInput potentiometerValue = new AnalogInput(potentiometerPort);

  // Initializes an AnalogPotentiometer on port "potentiometerValue"
  AnalogPotentiometer potentiometer = new AnalogPotentiometer(potentiometerValue, minLiftExtension, maxLiftExtension);

  public Lift() {

    // Enables 2-bit averaging
    potentiometerValue.setAverageBits(2);

    liftWinch.configFactoryDefault();
    liftWinch.setSafetyEnabled(true);
    liftWinch.setNeutralMode(NeutralMode.Brake);

    hookWinch.configFactoryDefault();
    hookWinch.setSafetyEnabled(true);
    hookWinch.setNeutralMode(NeutralMode.Brake);
    hookWinch2.configFactoryDefault();
    hookWinch2.setSafetyEnabled(true);
    hookWinch2.setNeutralMode(NeutralMode.Brake);
    hookWinch2.follow(hookWinch);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    isBrakeSet();
    liftExtension();
    isPinned();

  }

  public void setBrakeOn() {
    // Turns on the brake
  }

  public void setBrakeOff() {
    // Turns off the brake
  }

  public boolean isBrakeSet() {
    // Checks to see if the brake is set
    return true;
  }

  public double liftExtension() {

    // returns in the scale from minLiftExtention to maxLiftExtention
    return potentiometer.get();

  }

  public boolean isLiftFullyRetracted() {
    // Checks to see if the lift is fully retracted
    if (liftExtension() <= physicalMinExtension) {
      return true;
    } else {
      return false;
    }

  }

  public boolean isLiftFullyExtended() {
    // Checks to see if the lift is fully extended
    if (liftExtension() >= physicalMaxExtension) {
      return true;
    } else {
      return false;
    }

  }

  public void pinLift() {
    // Replaces the pin in the lift
  }

  public void unpinLift() {
    // Removes the pin in the lift
  }

  public boolean isPinned() {
    // Checks to see if the lift is pinned or not
    return true;
  }

  @Override
  public void setLiftPower(double percVal) {
    liftWinch.set(ControlMode.PercentOutput, percVal);
  }

  @Override
  public void setHookPower(double percVal) {
    hookWinch.set(ControlMode.PercentOutput, percVal);
  }
  
}
