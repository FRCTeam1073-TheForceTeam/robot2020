package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;

public class Drivetrain extends SubsystemBase {
    public WPI_TalonSRX left = RobotContainer.leftMotorLeader;
    public WPI_TalonSRX right = RobotContainer.rightMotorLeader;
    public PIDController pidLeft;
    public PIDController pidRight;
    public TestSubsystem() {
        pidLeft = new PIDController(0.1, 0, 0);
        pidRight = new PIDController(0.1, 0, 0);
    }

    /**It's a function! */
    @Override
    public void periodic() {
        //This method will be called once per sceduler run
        System.out.println("Periodic!");
    }

}