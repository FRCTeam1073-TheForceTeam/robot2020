/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    //the maximum velocity the drivetrain can drive at
    public static final double MAX_DRIVETRAIN_VELOCITY = 100.0;
    //the minimum distance needed to drive off of the initiation line
    public static final double MIN_DISTANCE_INIT_LINE = 0.46;
    //the maximum velocity the turret can turn at
    public static final double MAX_TURRET_VELOCITY = 100.0;
    //the maximum velocity the hood can move at
    public static final double MAX_HOOD_VELOCITY = 100.0;
}
