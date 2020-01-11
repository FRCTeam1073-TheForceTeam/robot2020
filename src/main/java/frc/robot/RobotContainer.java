package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;

public class RobotContainer {
    public static WPI_TalonSRX leftMotorLeader;
    public static WPI_TalonSRX rightMotorLeader;
    public static WPI_TalonSRX leftMotorFollower;
    public static WPI_TalonSRX rightMotorFollower;
    public static WPI_TalonSRX leftMotorFollower2;
    public static WPI_TalonSRX rightMotorFollower2;
    public static void init() {
        //FUn Fact: It's pronounced "ph-WHE-nix"
        
        leftMotorLeader = new WPI_TalonSRX(12);
        rightMotorLeader = new WPI_TalonSRX(13);
        leftMotorFollower = new WPI_TalonSRX(14);
        rightMotorFollower = new WPI_TalonSRX(15);
        leftMotorFollower2 = new WPI_TalonSRX(16);
        rightMotorFollower2 = new WPI_TalonSRX(17);
        
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
        leftMotorLeader.config_kP(0,10.7);
        rightMotorLeader.config_kP(0,10.7);
        leftMotorLeader.config_kI(0,0);
        rightMotorLeader.config_kI(0,0);
        leftMotorLeader.config_kD(0,0);
        rightMotorLeader.config_kD(0,0);

        leftMotorFollower.follow(leftMotorLeader);
        leftMotorFollower2.follow(leftMotorLeader);
        rightMotorFollower.follow(rightMotorLeader);
        rightMotorFollower2.follow(rightMotorLeader);
    }
}