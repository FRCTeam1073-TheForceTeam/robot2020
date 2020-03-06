package frc.robot.subsystems.instances;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.subsystems.interfaces.DrivetrainInterface;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class DrivetrainMercury extends SubsystemBase implements DrivetrainInterface {
    private ADXRS450_Gyro gyro;
    private DifferentialDriveOdometry odometry;

    private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.6477);
   //private double wheelDiameter = 0.15;
    // private double ticksPerWheelRotation =
    // ((52352+56574+54036+56452+53588+57594)/6.0)*0.1;//7942.8;
    private double ticksPerMeter = ((52352 + 56574 + 54036 + 56452 + 53588 + 57594) / 6.0) / Units.feetToMeters(10);
    // ticksPerWheelRotation / (Math.PI * wheelDiameter);
    private static WPI_TalonSRX leftMotorLeader;
    private static WPI_TalonSRX rightMotorLeader;
    private static WPI_VictorSPX leftMotorFollower;
    private static WPI_VictorSPX rightMotorFollower;
    private static WPI_VictorSPX leftMotorFollower2;
    private static WPI_VictorSPX rightMotorFollower2;

    private Pose2d robotPose = new Pose2d();
    private double gyroAngle = 0;

    public DrivetrainMercury() {
        // Setting up motors
        // FUn Fact: It's pronounced "ph-WHE-nix"

        leftMotorLeader = new WPI_TalonSRX(12);
        rightMotorLeader = new WPI_TalonSRX(13);
        leftMotorFollower = new WPI_VictorSPX(14);
        rightMotorFollower = new WPI_VictorSPX(15);
        leftMotorFollower2 = new WPI_VictorSPX(16);
        rightMotorFollower2 = new WPI_VictorSPX(17);

        leftMotorLeader.configFactoryDefault();
        rightMotorLeader.configFactoryDefault();
        leftMotorFollower.configFactoryDefault();
        rightMotorFollower.configFactoryDefault();
        leftMotorFollower2.configFactoryDefault();
        rightMotorFollower2.configFactoryDefault();

        leftMotorLeader.setSafetyEnabled(false);
        rightMotorLeader.setSafetyEnabled(false);
        leftMotorFollower.setSafetyEnabled(false);
        rightMotorFollower.setSafetyEnabled(false);
        leftMotorFollower2.setSafetyEnabled(false);
        rightMotorFollower2.setSafetyEnabled(false);

        leftMotorLeader.setNeutralMode(NeutralMode.Brake);
        rightMotorLeader.setNeutralMode(NeutralMode.Brake);
        leftMotorFollower.setNeutralMode(NeutralMode.Brake);
        rightMotorFollower.setNeutralMode(NeutralMode.Brake);
        leftMotorFollower2.setNeutralMode(NeutralMode.Brake);
        rightMotorFollower2.setNeutralMode(NeutralMode.Brake);

        leftMotorLeader.configPeakOutputForward(1.0);
        rightMotorLeader.configPeakOutputReverse(-1.0);
        leftMotorFollower.configPeakOutputForward(1.0);
        rightMotorFollower.configPeakOutputReverse(-1.0);
        leftMotorFollower2.configPeakOutputForward(1.0);
        rightMotorFollower2.configPeakOutputReverse(-1.0);

        leftMotorLeader.setInverted(true);
        leftMotorLeader.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        rightMotorLeader.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        leftMotorLeader.setSensorPhase(true);
        rightMotorLeader.setSensorPhase(true);
        double P = 2;
        double I = 0;
        double D = 6;
        leftMotorLeader.config_kP(0, P);
        rightMotorLeader.config_kP(0, P);
        leftMotorLeader.config_kI(0, I);
        rightMotorLeader.config_kI(0, I);
        leftMotorLeader.config_kD(0, D);
        rightMotorLeader.config_kD(0, D);

        leftMotorFollower.follow(leftMotorLeader);
        leftMotorFollower2.follow(leftMotorLeader);
        rightMotorFollower.follow(rightMotorLeader);
        rightMotorFollower2.follow(rightMotorLeader);

        gyro = new ADXRS450_Gyro();
        gyro.calibrate();
        odometry = new DifferentialDriveOdometry(getAngleRadians());
        leftMotorLeader.setSelectedSensorPosition(0);
        rightMotorLeader.setSelectedSensorPosition(0);

        /*SmartDashboard.putNumber("P", P);
        SmartDashboard.putNumber("I", I);
        SmartDashboard.putNumber("D", D);
        SmartDashboard.clearPersistent("P");
        SmartDashboard.clearPersistent("I");
        SmartDashboard.clearPersistent("D");*/
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
        // System.out.println("Meters: " + robotPose.getTranslation().getX() + "," + robotPose.getTranslation().getY()
        //         + "," + getAngleRadians());
        // System.out.println("Feet: " + Units.metersToFeet(robotPose.getTranslation().getX()) + ","
        //         + Units.metersToFeet(robotPose.getTranslation().getY()) + "," + getAngleRadians());
        // System.out.println("Periodic! " + getLeftEncoder() + ":" + getRightEncoder());
        /*SmartDashboard.putBoolean("hasStoppedRobot", hasRobotStopped);
        SmartDashboard.putNumber("rawGyroAngle", rawGyroAngle);
        SmartDashboard.putNumber("gyroDriftValue", gyroDriftValue);
        SmartDashboard.putNumber("totalGyroDrift", totalGyroDrift);
        SmartDashboard.putNumber("lastGyroValue", lastGyroValue);
        SmartDashboard.putNumber("gyroAngle", gyroAngle);
        SmartDashboard.putNumber("X", Units.metersToFeet(robotPose.getTranslation().getX()));
        SmartDashboard.putNumber("Y", Units.metersToFeet(robotPose.getTranslation().getY()));
        SmartDashboard.putNumber("Rotation", getAngleDegrees());
        SmartDashboard.putNumber("leftPower", leftPower);
        SmartDashboard.putNumber("rightPower", rightPower);*/
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
        leftMotorLeader.set(ControlMode.Velocity, left*3500);
        rightMotorLeader.set(ControlMode.Velocity, right*3500);
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

    @Override
    public void setRotationalVelocity(double left, double right) {
        // TODO Auto-generated method stub

    }

    public boolean isDrivetrainEngaged() {
        return true;
    }

    public void setWinchPower(double power) {
    }
}