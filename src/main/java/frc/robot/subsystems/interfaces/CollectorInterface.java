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
public interface CollectorInterface {

    void run(double speed, String direction);

    void collect();

    void purge();

    void raise();

    void lower();

    void stop();

    void lockIntake();

    void unlockIntake();
}
