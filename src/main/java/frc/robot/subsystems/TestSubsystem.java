package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;

public class TestSubsystem extends SubsystemBase {
    public WPI_TalonSRX left=RobotContainer.leftMotorLeader;
    public WPI_TalonSRX right=RobotContainer.rightMotorLeader;
    public TestSubsystem() {
    }

    /**It's a function! */
    @Override
    public void periodic() {
        //This method will be called once per sceduler run
        System.out.println("Periodic!");
    }

}