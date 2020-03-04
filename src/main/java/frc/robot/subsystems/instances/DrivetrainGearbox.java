package frc.robot.subsystems.instances;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.subsystems.interfaces.DrivetrainInterface;
import frc.robot.subsystems.interfaces.WinchInterface;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;

public class DrivetrainGearbox extends SubsystemBase implements DrivetrainInterface, WinchInterface {
    private static WPI_TalonFX leftMotorLeader;
    private static WPI_TalonFX leftMotorFollower;
    //private static LimitSwitchNormal limitSwitch;

    Solenoid solenoid = new Solenoid(1, 3);

    private boolean winchEngaged;

    public DrivetrainGearbox() {
        // Setting up motors
        // Fun Fact: It's pronounced "ph-WHE-nix"
        
        leftMotorLeader = new WPI_TalonFX(12);
        leftMotorFollower = new WPI_TalonFX(14);
        leftMotorLeader.configFactoryDefault();
        leftMotorFollower.configFactoryDefault();

        leftMotorFollower.follow(leftMotorLeader);
        leftMotorFollower.setInverted(true);
        solenoid.set(true);
        // engageDrivetrain();
    }

    /**
     * Returns the gyro feedback in degrees instead of radians so that humans
     * reading SmartDashboard feel at ease
     */

    public double getAngleDegrees() {
        // Rotrwation?
        return 0;
    }

    public Rotation2d getAngleRadians() {
        // Rotrwation?
        return Rotation2d.fromDegrees(0);
    }

    boolean hasRobotStopped = false;
    double gyroDriftValue = 0;
    double lastGyroValue = 0;
    double totalGyroDrift = 0;

    /** It's a function! */
    @Override
    public void periodic() {
    }

    public Pose2d getRobotPose() {
        return new Pose2d(0,0,new Rotation2d(0));
    }

    /**
     * Warning: resetting robot odometry will mean the robot will have ABSOLUTELY NO
     * IDEA where it is. Use with care.
     */
    public void resetRobotOdometry() {
    }

    private double leftPower = 0, rightPower = 0;

    public void setVelocity(double left, double right) {
        //System.out.println("x");
    }

    public void setPower(double left, double right) {
        leftMotorLeader.set(ControlMode.PercentOutput, left);
        //System.out.println("x");
    }

    /**
     * Returns linear wheel speeds.
     * 
     * @return Speed of left and right sides of the robot in meters per second.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(0, 0);
        // return new DifferentialDriveWheelSpeeds(leftMotorLeader.getSelectedSensorPosition() / ticksPerMeter,
        //         rightMotorLeader.getSelectedSensorPosition() / ticksPerMeter);
    }

    public double getLeftEncoder() {
        return leftMotorLeader.getSelectedSensorPosition();
    }

    public double getRightEncoder() {
        return 0;
        // return rightMotorLeader.getSelectedSensorPosition();
    }

    /**
     * Sets PID configurations
     */

    public void setPID(double P, double I, double D) {
        leftMotorLeader.config_kP(0, P);
        leftMotorLeader.config_kI(0, I);
        leftMotorLeader.config_kD(0, D);
    }

    public ChassisSpeeds getDrivetrainVelocity() {
        return new ChassisSpeeds(0, 0, 0);
        // kinematics.toChassisSpeeds(getWheelSpeeds());
    }

    public void engageWinch(){
        // leftMotorLeader.configFactoryDefault();
        // rightMotorLeader.configFactoryDefault();
        // leftMotorFollower.configFactoryDefault();
        // rightMotorFollower.configFactoryDefault();

        // // Keep this false for testing on roadkill where motors are unplugged
        // leftMotorLeader.setSafetyEnabled(false);
        // rightMotorLeader.setSafetyEnabled(false);
        // leftMotorFollower.setSafetyEnabled(false);
        // rightMotorFollower.setSafetyEnabled(false);

        // leftMotorLeader.setNeutralMode(NeutralMode.Brake);
        // rightMotorLeader.setNeutralMode(NeutralMode.Brake);
        // leftMotorFollower.setNeutralMode(NeutralMode.Brake);
        // rightMotorFollower.setNeutralMode(NeutralMode.Brake);

        // leftMotorLeader.configPeakOutputForward(1.0);
        // leftMotorFollower.configPeakOutputForward(1.0);
        // rightMotorLeader.configPeakOutputReverse(-1.0);
        // rightMotorFollower.configPeakOutputReverse(-1.0);

        // leftMotorLeader.setInverted(true);

        // leftMotorLeader.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        // rightMotorLeader.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

        // leftMotorLeader.setSensorPhase(true);
        // rightMotorLeader.setSensorPhase(true);

        // leftMotorFollower.follow(leftMotorLeader);
        // rightMotorFollower.follow(rightMotorLeader);
        
        // leftMotorLeader.setSelectedSensorPosition(0);
        // rightMotorLeader.setSelectedSensorPosition(0);

        // leftMotorLeader.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
        // leftMotorLeader.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);

        // rightMotorLeader.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
        // rightMotorLeader.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);

        // solenoid.set(true);

        // winchEngaged = true;
    }
    public void engageDrivetrain() {

        // leftMotorLeader.configFactoryDefault();
        // rightMotorLeader.configFactoryDefault();
        // leftMotorFollower.configFactoryDefault();
        // rightMotorFollower.configFactoryDefault();

        // leftMotorLeader.setSafetyEnabled(false);
        // rightMotorLeader.setSafetyEnabled(false);
        // leftMotorFollower.setSafetyEnabled(false);
        // rightMotorFollower.setSafetyEnabled(false);

        // leftMotorLeader.setNeutralMode(NeutralMode.Brake);
        // rightMotorLeader.setNeutralMode(NeutralMode.Brake);
        // leftMotorFollower.setNeutralMode(NeutralMode.Brake);
        // rightMotorFollower.setNeutralMode(NeutralMode.Brake);

        // leftMotorLeader.configPeakOutputForward(1.0);
        // leftMotorLeader.configPeakOutputReverse(-1.0);
        // leftMotorFollower.configPeakOutputForward(1.0);
        // leftMotorFollower.configPeakOutputReverse(-1.0);
        // rightMotorLeader.configPeakOutputForward(1.0);
        // rightMotorLeader.configPeakOutputReverse(-1.0);
        // rightMotorFollower.configPeakOutputForward(1.0);
        // rightMotorFollower.configPeakOutputReverse(-1.0);

        // leftMotorLeader.setInverted(true);
        // leftMotorLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        // rightMotorLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        // leftMotorLeader.setSensorPhase(true);
        // rightMotorLeader.setSensorPhase(true);
        // double P = 0.05;
        // double I = 0;
        // double D = 0;
        // leftMotorLeader.config_kP(0, P);
        // rightMotorLeader.config_kP(0, P);
        // leftMotorLeader.config_kI(0, I);
        // rightMotorLeader.config_kI(0, I);
        // leftMotorLeader.config_kD(0, D);
        // rightMotorLeader.config_kD(0, D);

        // leftMotorFollower.follow(leftMotorLeader);
        // leftMotorFollower.setInverted(true);
        // rightMotorFollower.follow(rightMotorLeader);
        // rightMotorFollower.setInverted(true);

        // leftMotorLeader.setSelectedSensorPosition(0);
        // rightMotorLeader.setSelectedSensorPosition(0);
        // leftMotorLeader.setIntegralAccumulator(0);
        // rightMotorLeader.setIntegralAccumulator(0);

        // solenoid.set(false);
        
        // winchEngaged = false;
    }

    public boolean isWinchEngaged(){
        return winchEngaged;
    }
    
    public int isFwdLimitSwitchClosedLeft() {
        return 0;
    }

    public int isRevLimitSwitchClosedLeft() {
        return 0;
    }

    public int isFwdLimitSwitchClosedRight() {
        return 0;
    }

    public int isRevLimitSwitchClosedRight() {
        return 0;
    }

    @Override
    public void setRotationalVelocity(double left, double right) {
        // TODO Auto-generated method stub

    }

    public void setWinchPower(double power) {
    }

    public boolean isDrivetrainEngaged() {
        return true;
    }
}