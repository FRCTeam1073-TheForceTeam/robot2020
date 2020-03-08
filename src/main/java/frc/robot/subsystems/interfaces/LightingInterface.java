/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.interfaces;

/**
 * Controls the port-tracking lights
 */
public interface LightingInterface {

    /**
     * gets brightness of the LEDs on the port-tracking OpenMV sensor module through RIO and PWM
     * @return double (0 - 1) 
     */
    public double getLEDLevel();

    /**
     * sets brightness of the LEDs on the port-tracking OpenMV sensor module through RIO and PWM
     * set double (0 - 1)
     */
    public void setLEDLevel(double LEDLevel);

}
