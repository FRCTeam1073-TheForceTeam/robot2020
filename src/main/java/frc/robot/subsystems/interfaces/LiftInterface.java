/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.interfaces;

public interface LiftInterface {

    // brake control
    void setBrakeOn();
    void setBrakeOff();
    boolean isBrakeSet();

    // potentiometer
    boolean isLiftFullyExtended();
    boolean isLiftFullyRetracted();
    double liftExtension();

    // pin
    void pinLift();
    void unpinLift();
    boolean isPinned();
    
    void setLiftPower(double percVal);

    void setHookPower(double percVal);
}
