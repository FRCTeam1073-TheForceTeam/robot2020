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
public interface WinchInterface {
    // I CHOOOOSE: Winch! Winch uses ENGAGE!    
    void engageWinch();

    void engageDrivetrain();
    
    boolean isWinchEngaged();

    public int isFwdLimitSwitchClosedLeft();

    public int isFwdLimitSwitchClosedRight();

    public int isRevLimitSwitchClosedLeft();

    public int isRevLimitSwitchClosedRight();
}
