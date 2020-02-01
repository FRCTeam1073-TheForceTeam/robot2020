/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.instances;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.TurretInterface;

public class Turret extends SubsystemBase implements TurretInterface {
  WPI_TalonSRX turretRotator;

  public Turret() {
  turretRotator = new WPI_TalonSRX(24);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

    /**
     * Return the timestamp of the last turret state update.
     * @return
     */
    @Override
    public long getLastTurretUpdate() {

      return 0;

    }

  @Override
  public boolean setPosition(double azimuth) {

    return false;

  }

  /**
   * Return the maximum turret angle in radians.
   * @return Maximum turret angle in radians.
   */
  @Override
  public double getMaxPosition() {

    return 1.0;

  }

  /**
   * Return the minimum turret angle in radians.
   * @return Minimum turret angle in radians.
   */
  @Override
  public double getMinPosition() {

    return -1.0;

  }

  /**
   * Set the turret rotation to a closed loop velocity given in radians/second. This command
   * works even if the turret has not been indexed.
   * 
   * @param angular_rate is rotation speed in radians/second.
   * @return True if turret is indexed, false if turret has not been indexed.
   */
  @Override
  public boolean setVelocity(double angular_rate) {

    return false;

  }

  /**
   * Disable the motor controls of the turret so that it will be 'limp'. 
   * Sending any position or velocity command will re-enable the turret.
   */
  @Override
  public void disable() {

  }

  /**
   * Return the azimuth angle of the turret in radians. This
   * value is only meaningful if isIndexed is true.
   * 
   * @return angle of the turret in Radians.
   */
  @Override
  public double getPosition() {

    return 0.0;

  }

  /**
   * Return the current turret velocity in radians / second.
   * @return angular rate of the turret in radians/second.
   */
  @Override
  public double getVelocity() {

    return 0.0;

  }

  /**
   * Return true if the turret has been indexed.
   * @return True if the turret has been indexed, false if it has not been indexed.
   */
  @Override
  public boolean isIndexed() {

    return false;
    
  }
  
}
