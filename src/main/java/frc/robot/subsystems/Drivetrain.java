package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.RobotContainer;

public class Drivetrain extends SubsystemBase {
    public WPI_TalonSRX left = RobotContainer.leftMotorLeader;
    public WPI_TalonSRX right = RobotContainer.rightMotorLeader;
    public PIDController pidLeft;
    public PIDController pidRight;
    public ADXRS450_Gyro gyro;
    DifferentialDriveOdometry odometry;
    double L = 0;

    double wheelDiameter = 0.15;
    double ticksPerWheelRotation = 7942.8;
    double ticksPerMeter = ticksPerWheelRotation / (Math.PI * wheelDiameter);

    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.65);

    public Drivetrain() {
        gyro = new ADXRS450_Gyro();
        pidLeft = new PIDController(0.1, 0, 0);
        pidRight = new PIDController(0.1, 0, 0);
        gyro.calibrate();
        odometry = new DifferentialDriveOdometry(getAngle());
        left.setSelectedSensorPosition(0);
        right.setSelectedSensorPosition(0);
        L = left.getSelectedSensorPosition();
    }

    public Rotation2d getAngle() {
        //Rotrwation?
        return Rotation2d.fromDegrees(-gyro.getAngle());
    }

    /**It's a function! */
    @Override
    public void periodic() {
        //This method will be called once per sceduler run
        // new DifferentialDriveWheelSpeeds()
        double leftSpeed = left.getSelectedSensorPosition() / ticksPerMeter;
        double rightSpeed = right.getSelectedSensorPosition() / ticksPerMeter;
        Pose2d pose = odometry.update(getAngle(), leftSpeed, rightSpeed);
        System.out.println(pose.getTranslation().getX() + "," + pose.getTranslation().getY()+","+getAngle());
        System.out.println("Periodic! "+(left.getSelectedSensorPosition()-L));
    }

}