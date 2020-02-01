/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.interfaces;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

/**
 * This interfcae is implemented by subsystems that provide odometry information
 * for use in localization subsystems. The Odometry source interface provides
 * the movement of differential wheel drive systems to other subsystems.
 */
public interface DifferentialOdometrySource {

    /**
     * Return the timestamp of the last odometry update.
     * 
     * @return
     */
    public long getLastOdometryUpdate();

    /**
     * Return left encoder value.
     * @return
     */
    public double getLeftEncoder();
       
    /**
     * Return right encoder value.
     * @return
     */
    public double getRightEncoder();

    /**
     * Return differential drive wheel speeds.
     * @return
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds();


    /**
     * Return current left odometry distance in meters.
     * @return
     */
    public double getLeftDistance();

    /**
     * Return current right odometry distance in meters.
     * @return
     */
    public double getRightDistance();

}
