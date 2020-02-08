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

    public DriveControls(DrivetrainInterface drivetrain_) {
        drivetrain = drivetrain_;
        addRequirements((SubsystemBase)drivetrain);
    }


    // starts the robot
    public void initialize() {

    }

    // executes actions defined here
    public void execute() {
        double amt = 1 - OI.driverController.getRawAxis(2);
        double fwd = deadzone(OI.driverController.getRawAxis(1) * amt);
        double rot = deadzone(OI.driverController.getRawAxis(4) * amt);
        drivetrain.setPower(fwd + rot, fwd - rot);
        if (OI.driverController.getAButtonPressed()) {
            drivetrain.resetRobotOdometry();
        }
    }
    
    public double deadzone(double val) {
        double zone = 0.1;
        if (Math.abs(val) < zone) {
            return 0;
        } else {
            return val;
        }
    }

    // ends the actions from execute when returned true
    public boolean isFinished() {
        return false;

    }
}
