/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.instances;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.ShooterInterface;

public class Shooter extends SubsystemBase implements ShooterInterface {
  private static WPI_TalonSRX shooterFlywheel1;
  private static WPI_TalonSRX shooterFlywheel2;

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    shooterFlywheel1 = new WPI_TalonSRX(22);
    shooterFlywheel2 = new WPI_TalonSRX(23);

    shooterFlywheel1.configFactoryDefault();
    shooterFlywheel2.configFactoryDefault();

    shooterFlywheel1.setSafetyEnabled(false);
    shooterFlywheel2.setSafetyEnabled(false);

    shooterFlywheel1.setNeutralMode(NeutralMode.Brake);
    shooterFlywheel2.setNeutralMode(NeutralMode.Brake);

    shooterFlywheel1.setInverted(false);
    shooterFlywheel2.setInverted(true);

    shooterFlywheel2.follow(shooterFlywheel1);

    shooterFlywheel1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    shooterFlywheel1.setSensorPhase(true);

    double P = 0;
    double I = 0;
    double D = 0;

    shooterFlywheel1.config_kP(0, P);
    shooterFlywheel1.config_kI(0, I);
    shooterFlywheel1.config_kD(0, D);

    shooterFlywheel1.setSelectedSensorPosition(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // TODO: Fill in what it needs...
    // Sample flywheel speed, hood position, flywheel temperature, etc.
  }

     /**
    * Return the timestamp of the last hood update.
    * @return
    */
    @Override
    public long getLastShooterUpdate() {
      // TODO: Return the last update of shooter state timestmap.
      return 0;
    }

    /**
     * Set the flywheel speed in radians/second. This sets a closed loop target velocity
     * for a flywheel and it will accelerate toward the target speed.
     * The speed will be clamped to the allowable speed limits.
     * @param speed in radians/second.
     */
    @Override
    public void setFlywheelSpeed(double speed) {
      
    }

    /**
     * Return the maximum flywheel speed.
     * @return Maximum flywheel speed in radians/second.
     */
    @Override
    public double getMaximumFlywheelSpeed() {
      return 11.0;
    }
 
    /**
     * Disable the motor axis of the flywheel so that it is limp. Sending a new speed command
     * will re-enable the flywheel.
     */
    @Override
    public void disableFlywheel() {

    }
 
    /**
     * Return the flywheel speed in radians/second.
     * @return Flywheel speed in radians/second.
     */
    @Override
    public double getFlywheelSpeed() {
      return 11.0;
    }
 
    /**
     * Return the flywheel motor internal temperature.
     * @return temperature in degrees C.
     */
    @Override
     public double[] getInternalTemperature() {
       return new double[2];
     }
 
    /**
     * Set the target hood angle. The hood will move toward this angle and hold this angle under
     * closed loop control. This command is only valid if the hood has been indexed.
     *
     * @param angle Angular postiion of the hood in radians.
     * @return True if the hood is indexed and we can set angles. False if the hood is not indexed.
     */
    @Override
    public boolean setHoodAngle(double angle) {
      return false;
    }

   /**
    * Set the hood to move in velocity mode which can be done without indexing the hood.
    * @param angle_rate
    * @return True if the hood is indexed, 
    */
    @Override
    public boolean setHoodVelocity(double angle_rate) {
      return false;
    }

    /**
     * Disable the hood axis control so that it is "limp". Setting a new hood angle will
     * re-enable the hood axis control.
     */
     @Override
     public void disableHood() {

     }
 
    /**
     * Return the current hood angle in radians.
     */
    @Override
    public double getHoodAngle() {
      return 0.0;
    }
 
    /**
     * Return the minimum allowed hood angle in radians.
     * @return
     */
    @Override
    public double getMinHoodAngle() {
      return 0.0;
    }
 
    /**
     * Return the maximum allowed hood angle in radians.
     * @return
     */
    @Override
    public double getMaxHoodAngle() {
      return 3.14;
    }

    @Override
    public boolean hoodIsIndexed() {
      // TODO: Return the hood index state.
      return false;
    }
}
