/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.instances;

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
}
