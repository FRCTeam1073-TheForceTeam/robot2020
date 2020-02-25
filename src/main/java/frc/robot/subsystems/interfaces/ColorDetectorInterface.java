/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.interfaces;

import frc.robot.subsystems.interfaces.WheelColor;

/**
 * This subsystem utilizes a color sensor to determine the control panel's
 * position and rotation count
 */
public interface ColorDetectorInterface {

    /**
     * Get the color over the sensor
     * @return Center color, in wheelColor enum
     */
    public WheelColor getCenterColor();

    /**
     * 
     * @return Coordinate of the center blob from 0-100% of image width
     */
    public int getCenterColorPosition();

    /**
     * 
     * @param color Color to rotate to, in wheelColor enum
     * @return Rotations of the control panel manipulator necessary to reach the target
     */
    public int getRotationsTo(WheelColor color);

    /**
     * 
     * @return Attempts since last camera update
     */
    public int getLastUpdateSeen();

    /**
     * 
     * @return Number of rotations of the control panel since last rotation init
     */
    public double getRotations();

    /**
     * Begin reading color and counting rotations
     */
    public void initRotations();
}
