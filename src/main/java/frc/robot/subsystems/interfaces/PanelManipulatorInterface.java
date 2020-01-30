/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.interfaces;

/**
 * Velocities all are in terms of wheel velocity and account for those ratios
 * This is assuming that we generally have full traction/contact
 */
public interface PanelManipulatorInterface {

    /**
     *
     * Sets horiz rotational velocity of Gladdys' wheels 
     * @param Velocity to set motor to must be between 0 and 1
     */
    public void setVelocity(double velocity);

    public double getMaxVelocity();

    /**
     * Uses encoder travel since last disable as value 
     */
    public double getRotation();

    /**
     * Returns last update time in milliseconds
     */
    public long getLastUpdateTime();

    /**
     * Disable the motor controls of Gladdys so that it will be non-moving 
     * Sending any position or velocity command will re-enable Gladdys
     * Resets the encoder position (setting to 0)
     */
    public void disable();



}
