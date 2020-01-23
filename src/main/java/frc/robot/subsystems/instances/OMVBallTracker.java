/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.instances;

import frc.robot.subsystems.interfaces.BallTrackerInterface;

public class OMVBallTracker extends OpenMVBase implements BallTrackerInterface{
  /**
   * Creates a new OMVBallTracker.
   */
  public OMVBallTracker(int deviceId, int deviceType) {
    super(deviceId, deviceType);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
