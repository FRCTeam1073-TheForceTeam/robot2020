/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.interfaces;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;


/**
 * This interface is provided by subsystems that provide localization of the
 * robot in either a global or local coordinate system.
 * 
 * The robot location is provided as (X,Y,Theta) both translation and rotation.
 * The location is provided in meters for distance and radians in angle.
 * 
 * Each localization interface represents the position of the robot in some
 * coordinate system. There can be many instances of localization that provide
 * locations in relative coordinates and in world coordinates. It depends on
 * when /where the localization interface is set to 0,0,0 (reset).
 * 
 * Implementations of this interface generally use DifferentialOdometrySource interface
 * as their input.
 */
public interface LocalizationInterface {

    /**
     * Return last update timestamp for this localization source.
     * @return
     */
    public long getLastLocalizationUpdate();

    /**
     * Return the current location of the robot for this subsystem in meters and radians as
     * a transformation.
     * 
     * @return
     */
    public Transform2d getTransform();

    /**
     * Return the current location as a pose2d.
     * 
     * @return
     */
    public Pose2d getPose();

    /**
     * Return overall drivetrain velocity.
     * @return
     */
    public ChassisSpeeds getDrivetrainVelocity();

    /**
     * Reset the location of this localization instance to 0,0,0
     */
    void zeroPose();

    /**
     * Reset the state of this localization source to the given pose.
     * @param init
     */
    void resetPose(Pose2d init);

}
