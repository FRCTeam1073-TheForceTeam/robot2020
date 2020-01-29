/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.interfaces;

/**
 * This interface is implemented by flywheel based shooter subsystems. The Flywheel
 * shooter provides control over a velocity-based flywheel and a position based hood
 * axis. It provides feedback on flywheel velocity and hood position and limits.
 * 
 */
public interface ShooterInterface {

   /**
    * Return the timestamp of the last hood update.
    * @return
    */
    public long getLastShooterUpdate();

    /**
     * Set the flywheel speed in radians/second. This sets a closed loop target velocity
     * for a flywheel and it will accelerate toward the target speed.
     * The speed will be clamped to the allowable speed limits.
     * @param speed in radians/second.
     */
   public void setFlywheelSpeed(double speed);

   /**
    * Return the maximum flywheel speed.
    * @return Maximum flywheel speed in radians/second.
    */
   public double getMaximumFlywheelSpeed();

   /**
    * Disable the motor axis of the flywheel so that it is limp. Sending a new speed command
    * will re-enable the flywheel.
    */
   public void disableFlywheel();

   /**
    * Return the flywheel speed in radians/second.
    * @return Flywheel speed in radians/second.
    */
   public double getFlywheelSpeed();

   /**
    * Return the flywheel motor internal temperature(s) as an array of doubles.
    * @return temperature in degrees C.
    */
    public double[] getInternalTemperature();

   /**
    * Set the target hood angle. The hood will move toward this angle and hold this angle under
    * closed loop control. This command is only valid if the hood has been indexed.
    * Returns true if the hood is indexed.
    *
    * @param angle Angular postiion of the hood in radians.
    */
   public boolean setHoodAngle(double angle);

   /**
    * Set the hood to move in velocity mode which can be done without indexing the hood.
    * @param angle_rate
    * @return True if the hood is indexed, 
    */
   public boolean setHoodVelocity(double angle_rate);

   /**
    * Disable the hood axis control so that it is "limp". Setting a new hood angle will
    * re-enable the hood axis control.
    */
    public void disableHood();

   /**
    * Return the current hood angle in radians.
    */
   public double getHoodAngle();

   /**
    * Return the minimum allowed hood angle in radians.
    * @return
    */
   public double getMinHoodAngle();

   /**
    * Return the maximum allowed hood angle in radians.
    * @return
    */
   public double getMaxHoodAngle();

   /**
    * Return true if the hood is indexed. False if the hood mechanism is not indexed.
    * Position commands are ignored until the hood is indexed.
    * @return
    */
   public boolean hoodIsIndexed();

}
