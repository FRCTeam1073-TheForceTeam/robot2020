/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.instances;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.CollectorInterface;

public class Collector extends SubsystemBase implements CollectorInterface {
  boolean isLocked = false;
  /**
   * Creates a new Collect.
   */
  public Collector() {

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
  
  }
  @Override
  public void lower(){

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
