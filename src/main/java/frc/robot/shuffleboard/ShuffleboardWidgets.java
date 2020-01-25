/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.shuffleboard;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

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
    //double P = 0.0;
    //double I = 0.0;
    //double D = 0.0;
    double drivetrainVelocity = 0.0;

    public ShuffleboardWidgets() {

      tab = Shuffleboard.getTab("Telemetry");

    }
  


  @Override
    public void periodic() {
      ShuffleboardInformation();
      ShuffleboardView();
    }
  
    private void ShuffleboardInformation() {

      // Drivetrain Data INCOMPLETE
      leftEncoderValue = Robot.subsystem.getLeftEncoder();
      rightEncoderValue = Robot.subsystem.getRightEncoder();
      
      // Gyro Datas
      gyroAngleDegrees = Robot.subsystem.getAngleDegrees();
      Pose2d pose = Robot.subsystem.getRobotPose();
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

}
