package frc.robot;
import com.ctre.phoenix.motorcontrol.can.*;
public class RobotContainer {
    public static WPI_TalonSRX leftMotorLeader;
    public static WPI_TalonSRX rightMotorLeader;
    public static WPI_TalonSRX leftMotorFollower;
    public static WPI_TalonSRX rightMotorFollower;
    public static WPI_TalonSRX leftMotorFollower2;
    public static WPI_TalonSRX rightMotorFollower2;
    public static void init() {
        //Phwenix
        leftMotorLeader=new WPI_TalonSRX(12);
        rightMotorLeader=new WPI_TalonSRX(13);
        leftMotorFollower=new WPI_TalonSRX(14);
        rightMotorFollower=new WPI_TalonSRX(15);
        leftMotorFollower2=new WPI_TalonSRX(16);
        rightMotorFollower2=new WPI_TalonSRX(17);
        leftMotorLeader.setInverted(true);
        leftMotorFollower.follow(leftMotorLeader);
        leftMotorFollower2.follow(leftMotorLeader);
        rightMotorFollower.follow(rightMotorLeader);
        rightMotorFollower2.follow(rightMotorLeader);
    }
}