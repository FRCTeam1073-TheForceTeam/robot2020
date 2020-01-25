/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.shuffleboard;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
//import frc.robot.Robot;
/**
 * Add your docs here.
 */
public class ShuffleboardInformation extends SubsystemBase {
    double leftEncoderValue;
    double rightEncoderValue;
    double gyroAngleDegrees;
    double robotX;
    double robotY;
    double robotRotation;
    double P;
    double I;
    double D;
    double drivetrainVelocity;

    public ShuffleboardInformation() {

    }
  
    @Override
    public void periodic() {
      sensorInputs();
    }
  
    public void sensorInputs() {

      // Drivetrain Data INCOMPLETE
      rightEncoderValue = Robot.subsystem.getRightEncoder();
      leftEncoderValue = Robot.subsystem.getLeftEncoder();

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
    
}
