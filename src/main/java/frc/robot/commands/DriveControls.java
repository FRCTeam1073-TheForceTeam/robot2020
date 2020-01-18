/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;

/**
 * Add your docs here.
 */
public class DriveControls extends CommandBase {
    Drivetrain subsystem;
    DifferentialDriveOdometry odometry;

    public DriveControls(Drivetrain subsystem_) {
        subsystem = subsystem_;
        addRequirements(subsystem);
    }

    // starts the robot
    public void initialize() {

    }

    // executes actions defined here
    public void execute() {
        double left = OI.driverController.getRawAxis(1);
        double right = OI.driverController.getRawAxis(5);
        subsystem.left.set(ControlMode.Velocity, left * 1500);
        subsystem.right.set(ControlMode.Velocity, right * 1500);
        System.out.println("A:"+subsystem.left.getClosedLoopError()+","+subsystem.left.getMotorOutputPercent());
        // subsystem.right.set(subsystem.pidRight.calculate(subsystem.left.getSelectedSensorVelocity(),OI.driverController.getRawAxis(1)));
        System.out.println("Hello cruel world");
    }

    // ends the actions from execute when returned true
    public boolean isFinished() {
        return false;

    }
}
