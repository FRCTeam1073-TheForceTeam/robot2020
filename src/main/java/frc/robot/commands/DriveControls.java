/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.interfaces.DrivetrainInterface;
import frc.robot.subsystems.interfaces.WinchInterface;
import frc.robot.Utility;

/**
 * Add your docs here.
 */
public class DriveControls extends CommandBase {

    DrivetrainInterface drivetrain;
    WinchInterface winch;
    private double deadzone = Constants.CONTROLLER_DEADZONE;
    private double multiplier;
    private double forward;
    private double rotation;
    private double rightOutput;
    private double leftOutput;

    /**
     * Sets the drive controls
     * @param drivetrain
     */
    public DriveControls(DrivetrainInterface drivetrain, WinchInterface winch) {
        this.drivetrain = drivetrain;
        this.winch = winch;
        addRequirements((SubsystemBase)drivetrain);
    }

    // starts the robot
    public void initialize() {
    }

    /**
     * adds the throttle multiplier to the axis value
     * @param axisValue
     * @param multiplier_
     * @return axisValue with throttle multiplier
     */
    private double addMultiplier(double axisValue, double multiplier_) {
        return axisValue * (0.125 + multiplier_ * 0.875);
    }


    /**
     * adds the default throttle multiplier to the axis value
     * @param axisValue
     * @return axisValue with the default throttle multiplier
     */
    private double addMultiplier(double axisValue) {
        return addMultiplier(axisValue, multiplier);
    }

    /**
     * ensures that the axis value are within the acceptable frame
     * @param axisValue
     * @return limited axisValue
     */
    private double limit(double axisValue) {
    	if (Math.abs(axisValue) > 1.0) {
            return Math.copySign(1, axisValue);
        }
    	return axisValue;
  	}

    /**
    * converts forward and rotation into Arcade Drive mode
    */
    private void arcadeCompute() {
        rotation *= -1;
        double maxInput = Math.copySign(Math.max(Math.abs(forward), Math.abs(rotation)), forward);
		if (forward >= 0.0) {
			if (rotation >= 0.0) {
				leftOutput = maxInput;
				rightOutput = forward - rotation;
			} else {
				leftOutput = forward + rotation;
				rightOutput = maxInput;
			}
		} else {
			if (rotation >= 0.0) {
				leftOutput = forward + rotation;
				rightOutput = maxInput;
			} else {
				leftOutput = maxInput;
				rightOutput = forward - rotation;
			}
		}
	}

    // executes actions defined here
    public void execute() {
        multiplier = Utility.deadzone(OI.driverController.getRawAxis(3));

        forward = addMultiplier(-Utility.deadzone(OI.driverController.getRawAxis(1)));
        rotation = addMultiplier(-Utility.deadzone(OI.driverController.getRawAxis(4)),0.5 * multiplier);

        rotation *= -1;

        if (drivetrain.isDrivetrainEngaged()) {
            arcadeCompute();
            // passes the final axis values into the drivetrain
            // drivetrain.setPower(limit(addMultiplier(leftOutput)), -limit(addMultiplier(rightOutput)));
            drivetrain.setVelocity(forward * 1.0, rotation);
        }


        if (winch.isWinchEngaged()) {
            winch.setWinchPower(addMultiplier(forward));
        }

        // ensures that the driver doesn't accidentally reset the odometry but makes it an option
        if (OI.driverController.getStartButtonPressed() && OI.driverController.getBackButtonPressed()) {
                drivetrain.resetRobotOdometry();
        }
    }

    // ends the actions from execute when returned true
    public boolean isFinished() {
        return false;
    }
}