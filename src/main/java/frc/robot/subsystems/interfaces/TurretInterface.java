/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.interfaces;

/**
 * This interface is implemented by turret control subsystems. A turret
 * subsystem must be indexed before it can return valid azimuth angles or
 * turn to a specific azimuth angle. Until it is indexed it does not know
 * where it is.
 */
public interface TurretInterface {

    /**
     * Return the timestamp of the last turret state update.
     * @return
     */
    public long getLastTurretUpdate();

    /**
     * Set the target pointing angle of the turret. It will move continuously toward this angle.
     * This command only works if the turret has been indexed and isIndexed returns true.
     * 
     * @param azimuth angle in radians.
     * @return True if position is set, false if turret has not been indexed.
     */
    public boolean setPosition(double azimuth);

    /**
     * Return the maximum turret angle in radians.
     * @return Maximum turret angle in radians.
     */
    public double getMaxPosition();

    /**
     * Return the minimum turret angle in radians.
     * @return Minimum turret angle in radians.
     */
    public double getMinPosition();

    /**
     * Set the turret rotation to a closed loop velocity given in radians/second. This command
     * works even if the turret has not been indexed.
     * 
     * @param angular_rate is rotation speed in radians/second.
     * @return True if turret is indexed, false if turret has not been indexed.
     */
    public boolean setVelocity(double angular_rate);

    /**
     * Disable the motor controls of the turret so that it will be 'limp'. 
     * Sending any position or velocity command will re-enable the turret.
     */
    public void disable();

    /**
     * Return the azimuth angle of the turret in radians. This
     * value is only meaningful if isIndexed is true.
     * 
     * @return angle of the turret in Radians.
     */
    public double getPosition();

    /**
     * Return the current turret velocity in radians / second.
     * @return angular rate of the turret in radians/second.
     */
    public double getVelocity();

    /**
     * Return true if the turret has been indexed.
     * @return True if the turret has been indexed, false if it has not been indexed.
     */
    public boolean isIndexed();

    public void deindex();

    /**
     * Resets turret encoder.
     */
    public void resetTurret();
}
