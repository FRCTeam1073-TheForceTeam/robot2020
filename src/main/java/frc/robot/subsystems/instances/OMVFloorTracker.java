/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.instances;

import frc.robot.subsystems.interfaces.SimpleTrackerInterface;
import edu.wpi.first.hal.CANData;
import frc.robot.subsystems.interfaces.LineTrackerInterface;

// Implementation for floor tracking cameras which track balls and line segments on floor,
public class OMVFloorTracker extends OpenMVBase implements SimpleTrackerInterface , LineTrackerInterface {
  private CANData targetData;
  private long lastTargetUpdate;
  private long lastLineUpdate;
  private SimpleTrackerInterface.TargetData[] targets;
  private LineTrackerInterface.LineData[] lines;

  private int lineLoopIncrement = 6;
  private int targetLoopIncrement = 6;

  /**
   * Creates a new OMVBallTracker.
   */
  public OMVFloorTracker(int deviceId) {
    super(deviceId);
    targetData = new CANData();
    targets = new SimpleTrackerInterface.TargetData[6];
    lines = new LineTrackerInterface.LineData[6];
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Call base class method
    super.periodic();
    
    if(readLines()){
      
    }
    
    if(readTargets()){

    }

  }

  private boolean readLines() {
    boolean foundLine = false;
    for(int i = 0; i < 6; i++){
      if (read(apiIndex(3, i), targetData) == true && targetData.length == 8){
        


        lines[i].quality = targetData.data[7];
        foundLine = true;

      }
    }

    if(foundLine){
      lineLoopIncrement = 0;
      return true;
    }

    // If the line searching has run more than 5 times without finding a line, it will return false and set
    // all line qualities to 0
    else
    {
      if(lineLoopIncrement++ >= 5){
        for (LineData lineData : lines) {
          lineData.quality = 0;
        }
      return false;
      }
      else return true;
    }
  }

  private boolean readTargets() {
    boolean foundTarget = false;
    for(int i = 0; i < 6; i++){
      if (read(apiIndex(2, i), targetData) == true && targetData.length == 8){


        
        foundTarget = true;

      }
    }

    if(foundTarget){
      targetLoopIncrement = 0;
      return true;
    }

    // If the line searching has run more than 5 times without finding a line, it will return false and set
    // all line qualities to 0
    else
    {
      if(targetLoopIncrement++ >= 5){
        for (TargetData targetData : targets) {
          targetData.quality = 0;
        }
      return false;
      }
      else return true;
    }
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
