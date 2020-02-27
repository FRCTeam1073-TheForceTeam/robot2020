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
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

public class Drivetrain extends SubsystemBase implements DrivetrainInterface, WinchInterface {
    private ADXRS450_Gyro gyro;
    private DifferentialDriveOdometry odometry;

    private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.6477);
   //private double wheelDiameter = 0.15;
    // private double ticksPerWheelRotation =
    // ((52352+56574+54036+56452+53588+57594)/6.0)*0.1;//7942.8;
    private double ticksPerMeter = ((52352 + 56574 + 54036 + 56452 + 53588 + 57594) / 6.0) / Units.feetToMeters(10);
    // ticksPerWheelRotation / (Math.PI * wheelDiameter);
    private static WPI_TalonFX leftMotorLeader;
    private static WPI_TalonFX rightMotorLeader;
    private static WPI_TalonFX leftMotorFollower;
    private static WPI_TalonFX rightMotorFollower;
    //private static LimitSwitchNormal limitSwitch;

    private Pose2d robotPose = new Pose2d();
    private double gyroAngle = 0;

    Solenoid solenoid = new Solenoid(6);

    private boolean winchEngaged;

    public Drivetrain() {
        // Setting up motors
        // Fun Fact: It's pronounced "ph-WHE-nix"
        
        leftMotorLeader = new WPI_TalonFX(12);
        rightMotorLeader = new WPI_TalonFX(13);
        leftMotorFollower = new WPI_TalonFX(14);
        rightMotorFollower = new WPI_TalonFX(15);

        gyro = new ADXRS450_Gyro();
        gyro.calibrate();
        odometry = new DifferentialDriveOdometry(getAngleRadians());

        engageDrivetrain();

    }

    /**
     * Returns the gyro feedback in degrees instead of radians so that humans
     * reading SmartDashboard feel at ease
     */

    public double getAngleDegrees() {
        // Rotrwation?
        return -gyroAngle;
    }

    public Rotation2d getAngleRadians() {
        // Rotrwation?
        return Rotation2d.fromDegrees(-gyroAngle);
    }

    boolean hasRobotStopped = false;
    double gyroDriftValue = 0;
    double lastGyroValue = 0;
    double totalGyroDrift = 0;

    /** It's a function! */
    @Override
    public void periodic() {
        // This method will be called once per sceduler run
        // new DifferentialDriveWheelSpeeds()
        double rawGyroAngle = gyro.getAngle();
        if (leftPower == 0 && rightPower == 0 && !hasRobotStopped) {
            hasRobotStopped = true;
            lastGyroValue = rawGyroAngle;
        }
        if ((leftPower != 0 || rightPower != 0) && hasRobotStopped) {
            totalGyroDrift += gyroDriftValue;
            hasRobotStopped = false;
            gyroAngle = 0;
        }
        if (hasRobotStopped) {
            gyroDriftValue = rawGyroAngle - lastGyroValue;
        }
        gyroAngle = rawGyroAngle - gyroDriftValue - totalGyroDrift;
        
        DifferentialDriveWheelSpeeds wheelSpeeds = getWheelSpeeds();
        robotPose = odometry.update(getAngleRadians(), wheelSpeeds.leftMetersPerSecond,
                wheelSpeeds.rightMetersPerSecond);
    }

    public Pose2d getRobotPose() {
        return robotPose;
    }

    /**
     * Warning: resetting robot odometry will mean the robot will have ABSOLUTELY NO
     * IDEA where it is. Use with care.
     */
    public void resetRobotOdometry() {
        odometry.resetPosition(new Pose2d(), getAngleRadians());
        robotPose = new Pose2d();
        leftMotorLeader.setSelectedSensorPosition(0);
        rightMotorLeader.setSelectedSensorPosition(0);
        gyro.reset();
        gyroAngle = 0;
        gyroDriftValue = 0;
        totalGyroDrift = 0;
        lastGyroValue = 0;
    }

    private double leftPower = 0, rightPower = 0;

    public void setVelocity(double left, double right) {
        leftMotorLeader.set(ControlMode.Velocity, left * 3500);
        rightMotorLeader.set(ControlMode.Velocity, right * 3500);
        leftPower = left;
        rightPower = right;
        //System.out.println("x");
    }

    public void setPower(double left, double right) {
        leftMotorLeader.set(ControlMode.PercentOutput, left);
        rightMotorLeader.set(ControlMode.PercentOutput, right);
        leftPower = left;
        rightPower = right;
        //System.out.println("x");
    }

    /**
     * Returns linear wheel speeds.
     * 
     * @return Speed of left and right sides of the robot in meters per second.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftMotorLeader.getSelectedSensorPosition() / ticksPerMeter,
                rightMotorLeader.getSelectedSensorPosition() / ticksPerMeter);
    }

    public double getLeftEncoder() {
        return leftMotorLeader.getSelectedSensorPosition();
    }

    public double getRightEncoder() {
        return rightMotorLeader.getSelectedSensorPosition();
    }

    /**
     * Sets PID configurations
     */

    public void setPID(double P, double I, double D) {
        leftMotorLeader.config_kP(0, P);
        leftMotorLeader.config_kI(0, I);
        leftMotorLeader.config_kD(0, D);
        rightMotorLeader.config_kP(0, P);
        rightMotorLeader.config_kI(0, I);
        rightMotorLeader.config_kD(0, D);
    }

    public ChassisSpeeds getDrivetrainVelocity() {
        return kinematics.toChassisSpeeds(getWheelSpeeds());
    }

    public void engageWinch(){
        leftMotorLeader.configFactoryDefault();
        rightMotorLeader.configFactoryDefault();
        leftMotorFollower.configFactoryDefault();
        rightMotorFollower.configFactoryDefault();

        // Keep this false for testing on roadkill where motors are unplugged
        leftMotorLeader.setSafetyEnabled(false);
        rightMotorLeader.setSafetyEnabled(false);
        leftMotorFollower.setSafetyEnabled(false);
        rightMotorFollower.setSafetyEnabled(false);

        leftMotorLeader.setNeutralMode(NeutralMode.Brake);
        rightMotorLeader.setNeutralMode(NeutralMode.Brake);
        leftMotorFollower.setNeutralMode(NeutralMode.Brake);
        rightMotorFollower.setNeutralMode(NeutralMode.Brake);

        leftMotorLeader.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 28, 33, 0.25));
        rightMotorLeader.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 28, 33, 0.25));
        leftMotorFollower.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 28, 33, 0.25));
        rightMotorFollower.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 28, 33, 0.25));

        leftMotorLeader.configPeakOutputForward(1.0);
        leftMotorFollower.configPeakOutputForward(1.0);
        rightMotorLeader.configPeakOutputReverse(-1.0);
        rightMotorFollower.configPeakOutputReverse(-1.0);

        leftMotorLeader.setInverted(true);

        leftMotorLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        rightMotorLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        leftMotorLeader.setSensorPhase(true);
        rightMotorLeader.setSensorPhase(true);

        leftMotorFollower.follow(leftMotorLeader);
        rightMotorFollower.follow(rightMotorLeader);
        
        leftMotorLeader.setSelectedSensorPosition(0);
        rightMotorLeader.setSelectedSensorPosition(0);

        leftMotorLeader.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
        leftMotorLeader.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);

        rightMotorLeader.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
        rightMotorLeader.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);

        solenoid.set(true);

        winchEngaged = true;
    }
    public void engageDrivetrain() {

        leftMotorLeader.configFactoryDefault();
        rightMotorLeader.configFactoryDefault();
        leftMotorFollower.configFactoryDefault();
        rightMotorFollower.configFactoryDefault();

        leftMotorLeader.setSafetyEnabled(false);
        rightMotorLeader.setSafetyEnabled(false);
        leftMotorFollower.setSafetyEnabled(false);
        rightMotorFollower.setSafetyEnabled(false);

        leftMotorLeader.setNeutralMode(NeutralMode.Brake);
        rightMotorLeader.setNeutralMode(NeutralMode.Brake);
        leftMotorFollower.setNeutralMode(NeutralMode.Brake);
        rightMotorFollower.setNeutralMode(NeutralMode.Brake);

        leftMotorLeader.configPeakOutputForward(1.0);
        leftMotorLeader.configPeakOutputReverse(-1.0);
        leftMotorFollower.configPeakOutputForward(1.0);
        leftMotorFollower.configPeakOutputReverse(-1.0);
        rightMotorLeader.configPeakOutputForward(1.0);
        rightMotorLeader.configPeakOutputReverse(-1.0);
        rightMotorFollower.configPeakOutputForward(1.0);
        rightMotorFollower.configPeakOutputReverse(-1.0);

        leftMotorLeader.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 28, 33, 0.25));
        rightMotorLeader.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 28, 33, 0.25));
        leftMotorFollower.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 28, 33, 0.25));
        rightMotorFollower.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 28, 33, 0.25));

        leftMotorLeader.setInverted(true);
        leftMotorLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        rightMotorLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        leftMotorLeader.setSensorPhase(true);
        rightMotorLeader.setSensorPhase(true);
        double P = 0.05;
        double I = 0;
        double D = 0;
        leftMotorLeader.config_kP(0, P);
        rightMotorLeader.config_kP(0, P);
        leftMotorLeader.config_kI(0, I);
        rightMotorLeader.config_kI(0, I);
        leftMotorLeader.config_kD(0, D);
        rightMotorLeader.config_kD(0, D);

        leftMotorFollower.follow(leftMotorLeader);
        leftMotorFollower.setInverted(true);
        rightMotorFollower.follow(rightMotorLeader);
        rightMotorFollower.setInverted(true);

        leftMotorLeader.setSelectedSensorPosition(0);
        rightMotorLeader.setSelectedSensorPosition(0);
        leftMotorLeader.setIntegralAccumulator(0);
        rightMotorLeader.setIntegralAccumulator(0);

        solenoid.set(false);
        
        winchEngaged = false;
    }

    public boolean isWinchEngaged(){
        return winchEngaged;
    }
    
    public int isFwdLimitSwitchClosedLeft() {
        return leftMotorLeader.isFwdLimitSwitchClosed();
    }

    public int isRevLimitSwitchClosedLeft() {
        return leftMotorLeader.isRevLimitSwitchClosed();
    }

    public int isFwdLimitSwitchClosedRight() {
        return rightMotorLeader.isFwdLimitSwitchClosed();
    }

    public int isRevLimitSwitchClosedRight() {
        return rightMotorLeader.isRevLimitSwitchClosed();
    }
}