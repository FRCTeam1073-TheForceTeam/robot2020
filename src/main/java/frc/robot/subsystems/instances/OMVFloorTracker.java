/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.instances;

import frc.robot.subsystems.interfaces.SimpleTrackerInterface;
import frc.robot.subsystems.interfaces.LineTrackerInterface;

// Implementation for floor tracking cameras which track balls and line segments on floor,
public class OMVFloorTracker extends OpenMVBase implements SimpleTrackerInterface , LineTrackerInterface {

  private long lastTargetUpdate;
  private long lastLineUpdate;
  private SimpleTrackerInterface.TargetData[] targets;
  private LineTrackerInterface.LineData[] lines;

  /**
   * Creates a new OMVBallTracker.
   */
  public OMVFloorTracker(int deviceId) {
    super(deviceId);
    targets = new SimpleTrackerInterface.TargetData[6];
    lines = new LineTrackerInterface.LineData[6];
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Call base class method
    super.periodic();
    
    // TODO: Call read and parsing methods.
  }

  @Override
  public SimpleTrackerInterface.TargetData[] getSimpleTargets() {
    return targets;
  }

  @Override
  public long getLastSimpleTargetUpdate() {
    return lastTargetUpdate;
  }

  @Override
  public LineTrackerInterface.LineData[] getLineData() {
    return lines;
  }

  @Override
  public long getLastLineDataUpdate() {
    return lastLineUpdate;
  }
}
