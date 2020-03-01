package frc.robot.subsystems.interfaces;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;


public interface DrivetrainInterface {

    public double getAngleDegrees();

    public Rotation2d getAngleRadians();

    public Pose2d getRobotPose();

    public void resetRobotOdometry();

    public void setRotationalVelocity(double left, double right);

    public void setVelocity(double forward, double rotation);

    public void setPower(double left, double right);

    public DifferentialDriveWheelSpeeds getWheelSpeeds();
       
    public double getLeftEncoder();
       
    public double getRightEncoder();
        
    public void setPID(double P, double I, double D);
       
    public ChassisSpeeds getDrivetrainVelocity();

    public double[] getOrientation();
}