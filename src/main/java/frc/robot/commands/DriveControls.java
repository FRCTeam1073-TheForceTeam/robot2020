/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.OI;
import frc.robot.subsystems.interfaces.DrivetrainInterface;

/**
 * Add your docs here.
 */
public class DriveControls extends CommandBase {

    DrivetrainInterface drivetrain;
    private double amount;
    private double forward;
    private double rotation;
    private double value;

    public DriveControls(DrivetrainInterface drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements((SubsystemBase)drivetrain);
    }

    // starts the robot
    public void initialize() {
    }

    // executes actions defined here
    public void execute() {
        amount = OI.driverController.getRawAxis(2) * 0.75;
        forward = deadzone(OI.driverController.getRawAxis(1) * (0.25 + amount));
        rotation = deadzone(OI.driverController.getRawAxis(4) * (0.25 + amount));
        drivetrain.setPower(forward + rotation, forward - rotation);
        /*if (OI.driverController.getAButtonPressed()) {
            drivetrain.resetRobotOdometry();
        }*/
    }
    
    public double deadzone(double value) {
        double zone = 0.1;
        if (Math.abs(value) < zone) {
            return 0;
        } else {
            return value;
        }
    }

    // ends the actions from execute when returned true
    public boolean isFinished() {
        return false;
    }
}