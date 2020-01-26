/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.instances;

import frc.robot.subsystems.interfaces.BallTrackerInterface;
import frc.robot.subsystems.interfaces.LineTrackerInterface;

// Implementation for floor tracking cameras which track balls and line segments on floor,
public class OMVFloorTracker extends OpenMVBase implements BallTrackerInterface{
  /**
   * Creates a new OMVBallTracker.
   */
  public OMVFloorTracker(int deviceId) {
    super(deviceId);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Call base class method
    super.periodic();
    
  }
}
