package frc.robot.subsystems.instances;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.OI;
import frc.robot.subsystems.interfaces.DrivetrainInterface;
import frc.robot.subsystems.interfaces.WinchInterface;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

public class Drivetrain extends SubsystemBase implements DrivetrainInterface, WinchInterface {
    private PigeonIMU gyro;
    private DifferentialDriveOdometry odometry;

    private double robotWidth = 0.5969;

    private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(robotWidth);
   //private double wheelDiameter = 0.15;
    // private double ticksPerWheelRotation =
    // ((52352+56574+54036+56452+53588+57594)/6.0)*0.1;//7942.8;
    private double ticksPerMeter = (
    (double) (
            //Raw Encoder Ticks
            /*Left    Right*/
            119300 + 119130 +
            118144 + 117828 +
            114665 + 119048 +
            120111 + 119625 +
            119420 + 119625
        ) / 10.0
    ) / Units.feetToMeters(10.0);
    // ticksPerWheelRotation / (Math.PI * wheelDiameter);
    private static WPI_TalonFX leftMotorLeader;
    private static WPI_TalonFX rightMotorLeader;
    private static WPI_TalonFX leftMotorFollower;
    private static WPI_TalonFX rightMotorFollower;
    private final double wheelDiameter = Units.inchesToMeters(5.9);
    //private static LimitSwitchNormal limitSwitch;

    private Pose2d robotPose = new Pose2d();
    private double gyroAngle = 0;

    Solenoid winch = new Solenoid(1, 1);
    Solenoid drivetrain = new Solenoid(1, 7);

    private boolean winchEngaged = false;

    public Drivetrain() {
        // Setting up motors
        // Fun Fact: It's pronounced "ph-WHE-nix"
        
        leftMotorLeader = new WPI_TalonFX(12);
        leftMotorFollower = new WPI_TalonFX(14);
        rightMotorLeader = new WPI_TalonFX(13);
        rightMotorFollower = new WPI_TalonFX(15);

        gyro = new PigeonIMU(6);
        gyro.setFusedHeading(0);
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

    /**
     * Returns angle in radians, as is proper.
     */
    public Rotation2d getAngleRadians() {
        // Rotrwation?
        return Rotation2d.fromDegrees(-gyroAngle);
    }

    boolean hasRobotStopped = false;
    double gyroDriftValue = 0;
    double lastGyroValue = 0;
    double totalGyroDrift = 0;

    @Override
    public void periodic() {
        // This method will be called once per sceduler run
        // new DifferentialDriveWheelSpeeds()
        double rawGyroAngle = gyro.getFusedHeading();
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
        
        // TODO: See what controls are actually going to be used

        // if (OI.driverController.getYButtonPressed()) {
        //     engageWinch();
        // }
        
        if (OI.driverController.getAButtonPressed()) {
            engageDrivetrain();
        }

        SmartDashboard.putNumber("Left Velocity",
                leftMotorLeader.getSelectedSensorVelocity() / 2048.0 * 10.0 * (2.0 * Math.PI));
        SmartDashboard.putNumber("Left Power", leftPower);
        SmartDashboard.putNumber("Left Output", leftMotorLeader.getMotorOutputPercent());
        SmartDashboard.putNumber("Left Error P", leftMotorLeader.getClosedLoopError());
        SmartDashboard.putNumber("Left Position", leftMotorLeader.getSelectedSensorPosition());
        SmartDashboard.putNumber("Right Position", rightMotorLeader.getSelectedSensorPosition());
        SmartDashboard.putBoolean("Winch Engaged", isWinchEngaged());
        SmartDashboard.putString("Coordinates",
                "(" + Math.round(
                        (getRobotPose().getTranslation().getX()
        )*1000.0)*0.001 + ","
                + Math.round(getRobotPose().getTranslation().getY()*1000.0)*0.001 + ")");
    }

    /**
     * Returns a Pose2d object containing the translation and rotation components of the robot's position.
     */
    public Pose2d getRobotPose() {
        return robotPose;
    }

    /**
     * Warning: resetting robot odometry will mean the robot will have ABSOLUTELY NO
     * IDEA where it is. At all. Use with care.
     */
    public void resetRobotOdometry() {
        odometry.resetPosition(new Pose2d(), getAngleRadians());
        robotPose = new Pose2d();
        leftMotorLeader.setSelectedSensorPosition(0);
        rightMotorLeader.setSelectedSensorPosition(0);
        gyro.setFusedHeading(0);
        gyroAngle = 0;
        gyroDriftValue = 0;
        totalGyroDrift = 0;
        lastGyroValue = 0;
    }

    private double leftPower = 0, rightPower = 0;

    /**
     * Sets motor rotational speeds in radians/second.
     * @param left The left motor speed.
     * @param right The right motor speed.
     */
    public void setRotationalVelocity(double left, double right) {
        leftMotorLeader.set(ControlMode.Velocity, left * 2048.0 * 0.1 / (2.0 * Math.PI));
        rightMotorLeader.set(ControlMode.Velocity, right * 2048.0 * 0.1 / (2.0 * Math.PI));
        leftPower = left;
        rightPower = right;
        //System.out.println("x");
    }

    /**
     * Sets the robot velocity.
     * @param forward The forward speed in meters/second
     * @param rotation The robot's rotational speed in radians/second
     */
    public void setVelocity(double forward, double rotation) {

        SmartDashboard.putNumber("SetVelocity Forward", forward);
        DifferentialDriveWheelSpeeds diffSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(forward, 0, rotation));
        double leftRotationalSpeed = diffSpeeds.leftMetersPerSecond / wheelDiameter;
        double rightRotationalSpeed = diffSpeeds.rightMetersPerSecond / wheelDiameter;

        SmartDashboard.putNumber("Left Set Power", leftRotationalSpeed * 2048.0 * 0.1 / (2.0 * Math.PI));
        leftMotorLeader.set(ControlMode.Velocity, leftRotationalSpeed * 2048.0 * 0.1 / (2.0 * Math.PI));
        rightMotorLeader.set(ControlMode.Velocity, -rightRotationalSpeed * 2048.0 * 0.1 / (2.0 * Math.PI));
        leftPower = leftRotationalSpeed;
        rightPower = rightRotationalSpeed;

        //System.out.println("x");
    }

    public void setPower(double left, double right) {
        leftMotorLeader.set(ControlMode.PercentOutput, left);
        rightMotorLeader.set(ControlMode.PercentOutput, right);
        leftPower = left;
        rightPower = right;
        System.out.println("x");
    }

    /**
     * Returns linear wheel speeds.
     * 
     * @return Speed of left and right sides of the robot in meters per second.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftMotorLeader.getSelectedSensorPosition() / ticksPerMeter,
                -rightMotorLeader.getSelectedSensorPosition() / ticksPerMeter);
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
        
        leftMotorLeader.neutralOutput();
        leftMotorFollower.neutralOutput();
        rightMotorLeader.neutralOutput();
        rightMotorFollower.neutralOutput();

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
        leftMotorLeader.configPeakOutputReverse(-1.0);
        leftMotorFollower.configPeakOutputForward(1.0);
        leftMotorFollower.configPeakOutputReverse(-1.0);
        rightMotorLeader.configPeakOutputForward(1.0);
        rightMotorLeader.configPeakOutputReverse(-1.0);
        rightMotorFollower.configPeakOutputForward(1.0);
        rightMotorFollower.configPeakOutputReverse(-1.0);

        leftMotorLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        rightMotorLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        leftMotorLeader.setSensorPhase(true);
        rightMotorLeader.setSensorPhase(true);

        leftMotorFollower.follow(leftMotorLeader);
        rightMotorLeader.follow(leftMotorLeader);
        rightMotorFollower.follow(leftMotorLeader);

        leftMotorLeader.setInverted(TalonFXInvertType.CounterClockwise);
        leftMotorFollower.setInverted(TalonFXInvertType.CounterClockwise);
        rightMotorLeader.setInverted(TalonFXInvertType.Clockwise);
        rightMotorFollower.setInverted(TalonFXInvertType.Clockwise);
        
        leftMotorLeader.setSelectedSensorPosition(0);
        rightMotorLeader.setSelectedSensorPosition(0);
        leftMotorLeader.setIntegralAccumulator(0);
        rightMotorLeader.setIntegralAccumulator(0);

        // leftMotorLeader.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
        // leftMotorLeader.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);

        // rightMotorLeader.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
        // rightMotorLeader.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);

        winch.set(true);
        drivetrain.set(false);

        winchEngaged = true;
    }
    public void engageDrivetrain() {

        leftMotorLeader.configFactoryDefault();
        rightMotorLeader.configFactoryDefault();
        leftMotorFollower.configFactoryDefault();
        rightMotorFollower.configFactoryDefault();

        leftMotorLeader.neutralOutput();
        leftMotorFollower.neutralOutput();
        rightMotorLeader.neutralOutput();
        rightMotorFollower.neutralOutput();

        leftMotorLeader.setSafetyEnabled(false);
        rightMotorLeader.setSafetyEnabled(false);
        leftMotorFollower.setSafetyEnabled(false);
        rightMotorFollower.setSafetyEnabled(false);

        leftMotorLeader.setNeutralMode(NeutralMode.Brake);
        rightMotorLeader.setNeutralMode(NeutralMode.Brake);
        leftMotorFollower.setNeutralMode(NeutralMode.Brake);
        rightMotorFollower.setNeutralMode(NeutralMode.Brake);

        leftMotorLeader.configClosedloopRamp(0.25);
        rightMotorLeader.configClosedloopRamp(0.25);

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

        leftMotorLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        rightMotorLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        leftMotorLeader.setSensorPhase(true);
        rightMotorLeader.setSensorPhase(true);
        double P = 1.1e-1;
        double I = 1e-2;
        double D = 0;
        double F = 0.75 * 0.138 * 1023 / 1750;

        leftMotorLeader.config_kP(0, P);
        rightMotorLeader.config_kP(0, P);
        leftMotorLeader.config_kI(0, I);
        rightMotorLeader.config_kI(0, I);
        leftMotorLeader.config_kD(0, D);
        rightMotorLeader.config_kD(0, D);
        leftMotorLeader.config_kF(0, F);
        rightMotorLeader.config_kF(0, F);

        leftMotorLeader.configMaxIntegralAccumulator(0, 400);
        rightMotorLeader.configMaxIntegralAccumulator(0, 400);

        leftMotorFollower.follow(leftMotorLeader);
        rightMotorFollower.follow(rightMotorLeader);

        leftMotorFollower.setInverted(TalonFXInvertType.CounterClockwise);
        rightMotorFollower.setInverted(TalonFXInvertType.CounterClockwise);
        leftMotorLeader.setInverted(TalonFXInvertType.CounterClockwise);
        rightMotorLeader.setInverted(TalonFXInvertType.CounterClockwise);

        leftMotorLeader.setSelectedSensorPosition(0);
        rightMotorLeader.setSelectedSensorPosition(0);
        leftMotorLeader.setIntegralAccumulator(0);
        rightMotorLeader.setIntegralAccumulator(0);

        winch.set(false);
        drivetrain.set(true);
        
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

    @Override
    public double[] getOrientation() {
        double[] orientation = { 0, 0, 0 };
        gyro.getYawPitchRoll(orientation);
        return orientation;
    }

    public void setWinchPower(double power) {
        leftMotorLeader.set(power);
    }

    public boolean isDrivetrainEngaged() {
        return !winchEngaged;
    }
}
