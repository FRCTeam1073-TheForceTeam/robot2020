/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.shuffleboard;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.interfaces.DrivetrainInterface;

//import frc.robot.Robot;
/**
 * Add your docs here.
 */
public class ShuffleboardWidgets extends SubsystemBase {

  ShuffleboardTab tab;

  double leftEncoderValue = 0.0;
  double rightEncoderValue = 0.0;

  double gyroAngleDegrees = 0.0;
  double robotX = 0.0;
  double robotY = 0.0;
  double robotRotation = 0.0;

  double drivetrainVelocity = 0.0;

  private NetworkTableEntry P_testing;
  private NetworkTableEntry I_testing;
  private NetworkTableEntry D_testing;
  double P_Value;
  double I_Value;
  double D_Value;

  DrivetrainInterface drivetrain;

    public ShuffleboardWidgets() {

      tab = Shuffleboard.getTab("Telemetry");

      P_testing = tab
      .add("P", 1)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 0, "max", 10))
      .getEntry();

      I_testing = tab
      .add("I", 0.01)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 0, "max", 0.1))
      .getEntry();

      D_testing = tab
      .add("D", 10)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 0, "max", 100))
      .getEntry();

      drivetrain = Robot.drivetrain;
      
    }
  
  @Override
    public void periodic() {
      ShuffleboardInformation();
      ShuffleboardView();
      PID_testing();

    }
  
    private void ShuffleboardInformation() {

      // Drivetrain Data INCOMPLETE
      leftEncoderValue = drivetrain.getLeftEncoder();
      rightEncoderValue = drivetrain.getRightEncoder();
      
      // Gyro Datas COMPLETE
      gyroAngleDegrees = drivetrain.getAngleDegrees();
      Pose2d pose = drivetrain.getRobotPose();
      robotX = pose.getTranslation().getX();
      robotY = pose.getTranslation().getY();
      robotRotation = pose.getRotation().getDegrees();

      // Turret Data MISSING

      // Shooter Data MISSING

      // Magazine Data MISSING

      // Climbing Data MISSING

      // Wheel of Fortune Data MISSING

    } 

    private void ShuffleboardView() {

      tab.add("leftEncoder", leftEncoderValue);
      tab.add("rightEncoder", rightEncoderValue);
      
      tab.add("gyroAngle", gyroAngleDegrees);
      tab.add("x-coordinate",robotX);
      tab.add("y-coordinate",robotY);
      tab.add("rotation",robotRotation);
      
    }

    public void PID_testing() {

      P_Value = P_testing.getDouble(1.0);
      I_Value = I_testing.getDouble(0.01);
      D_Value = D_testing.getDouble(10.0);

      //set values
      drivetrain.setPID(P_Value, I_Value, D_Value);

    }

}
