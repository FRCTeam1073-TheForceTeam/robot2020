/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.instances;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.WheelColor;
import frc.robot.subsystems.interfaces.ColorDetectorInterface;

public class ColorDetector extends OpenMVBase implements ColorDetectorInterface {
  int[] wheelColorCounts;
  WheelColor currColor;


  /**
   * Creates a new ColorDetector.
   */
  public ColorDetector(int deviceID) {
    super(deviceID);
    wheelColorCounts = new int[4];
    currColor = WheelColor.NULL;
  }

  @Override
  public void periodic() {
    super.periodic();
    // This method will be called once per scheduler run
  }

  @Override
  public WheelColor getCenterColor() {
    return currColor;
  }

  @Override
  public int getCenterColorPosition() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public int getRotationsTo(WheelColor color) {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public int getLastUpdateSeen() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public double getRotations() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public void initRotations() {
    wheelColorCounts = new int[4];
  }
}
