/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.interfaces;

/**
 * Add your docs here.
 */
public interface ShooterInterface {

    void setSpeed(double speed);

    void increaseSpeed(double speed);

    void decreaseSpeed(double speed);

    void setPID(double P, double I, double D);

    double getSpeed();

}
