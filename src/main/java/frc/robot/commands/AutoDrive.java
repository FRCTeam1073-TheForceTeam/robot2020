/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.subsystems.interfaces.DrivetrainInterface;

/**
 * Add your docs here.
 */
public class AutoDrive extends CommandBase {
    DrivetrainInterface drivetrain;
    private double velocity = 0.0;
    private double distance = Units.inchesToMeters(18);
    private boolean isFinished = false;
    private double distanceTraveled = 0.0;
    private int time = 0;

    public AutoDrive(DrivetrainInterface drivetrain_, double velocity, final double distance) {
        drivetrain = drivetrain_;
        this.velocity = velocity;
        this.distance = distance;

        addRequirements((SubsystemBase)drivetrain);
    }


    // starts the robot
    @Override
    public void initialize() {
        drivetrain.resetRobotOdometry();
    }

    // executes actions defined here
    @Override
    public void execute() {
        if (time <= 200) {
            drivetrain.setVelocity(velocity, velocity);
            Robot.bling.setPatternRGBAll(255, 0, 0);
        } else if (time <= 400) {
            drivetrain.setVelocity(-velocity, -velocity);
            Robot.bling.setPatternRGBAll(0, 255, 0);
        } else {
            Robot.bling.setPatternRGBAll(0, 0, 0);
            isFinished = true;
        }
        time = time + 1;

        // TODO: set up distances for auto
        //distanceTraveled = (drivetrain.getRobotPose()).getTranslation().getY();
        //if (distanceTraveled >= distance) {
        //drivetrain.setVelocity(0, 0);
            //
        //}
    }
    
    

    // ends the actions from execute when returned true
    @Override
    public boolean isFinished() {
        return isFinished;
    }


    public void end(){
    }
}
