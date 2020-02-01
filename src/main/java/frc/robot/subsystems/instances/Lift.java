/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.instances;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.LiftInterface;

public class Lift extends SubsystemBase implements LiftInterface {
  /**
   * Creates a new Lift.
   */
  private static Solenoid solenoid;

  // that is the measured length if the voltage from the potentiometer is 0 (the minimum)
  double minLiftExtention = 0.0;

  // that is the measured length if the voltage from the potentiometer is 1 (the maximum)
  double maxLiftExtention = 1.0;

  // defines the potentiometerPort
  int potentiometerPort = 0;

  public Lift() {
    solenoid = new Solenoid(17);

    // Initializes an AnalogInput on port "potentiometerValue", and enables 2-bit averaging
    AnalogInput potentiometerValue = new AnalogInput(potentiometerPort);
  
    // 
    AnalogPotentiometer potentiometer = new AnalogPotentiometer(potentiometerValue, minLiftExtention, maxLiftExtention);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    liftExtension();



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

  public boolean isLiftFullyExtended() {
    // Checks to see if the lift is fully extended
    return true;
  }

  public boolean isLiftFullyRetracted() {
    // Checks to see if the lift is fully retracted
    return true;
  }

  public double liftPosition() {
    // Returns the current position of the lift
    return 0;
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

  public void potentiometer() {

    potentiometerValue.setAverageBits(2);
  }

  public double liftExtension() {

    potentiometer.get();
    return 0.0;

  }

}
