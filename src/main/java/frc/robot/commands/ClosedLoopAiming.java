/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.AdvancedTrackerInterface;
import frc.robot.subsystems.interfaces.LightingInterface;
import frc.robot.subsystems.interfaces.ShooterInterface;
import frc.robot.subsystems.interfaces.TurretInterface;

public class ClosedLoopAiming extends CommandBase {

  public enum CLAMode {
    VELOCITY, POSITION
  }

  TurretInterface turret;
  AdvancedTrackerInterface portTracker;
  ShooterInterface shooter;
  LightingInterface lights;
  AdvancedTrackerInterface.AdvancedTargetData currentTarget;

  private String trackerStatus;

  private double azimuthTarget;
  private boolean temporary;

  private double azimuthCorrection = 0.0;
  private double accelConstant = 1.5;
  private double azimuthThreshold = 0.0;
  private double shooterVelocity = 0.0;
  private double hoodAngle = 0.0; 
  private TrajectoryConfiguration targetTrajectory;

  private CLAMode mode = CLAMode.VELOCITY;
  /**
   * When activated, will align turret and shooter to the port vision target
   * @param turret_ Turret susbsytem on the robot
   * @param portTracker_ Vision tracker for the power port
   * @param shooter_ Shooter subsystem on the robot
   * @param lights_ Lights control subsytem on the robot
   * @param shooter_ Shooter subsystem on the robot
   * @param temporary_ Set true to end the command when difference < success threshold
   */

  public ClosedLoopAiming(final TurretInterface turret_, final AdvancedTrackerInterface portTracker_,
      final ShooterInterface shooter_, final LightingInterface lights_, CLAMode mode_, boolean temporary_, double azimuthThreshold_) {
    turret = turret_;
    portTracker = portTracker_;
    shooter = shooter_;
    lights = lights_;
    mode = mode_;
    temporary = temporary_;
    azimuthThreshold = azimuthThreshold_;
    trackerStatus = "";
    addRequirements((SubsystemBase) turret, (SubsystemBase) portTracker);
  }

  public static final double kGravity = 9.807;
  public static final double kShooterHeight = 0.873;
  public static final double kRelativePortDistance = 0.873;
  private static final double kPowerCellRadius = 0.09;
  private static final double kFlywheelRadius = Units.inchesToMeters(2.5);
  //The distance after which the robot decides to switch to targeting only the outer port.
  public static final double distanceTargetingThreshold = 10;
 
  /**Gets velocity given the angle, range, and height.
   * @param angle The angle in radians.
   * @param portRange The range of the outer port horizontally.
   * @param portHeight The height of the outer port in meters.
   * @return The velocity in meters/second.
   */
  public double getVelocity(double angle, double portRange, double portHeight) {
    /*
     * The function is
     * 
     *                g*X^2*(1+tan(theta)^2)
     * V(theta)=sqrt(――――――――――――――――――――――――)
     *                2*(y_0-Y+X*tan(theta))
     */
    return Math.sqrt(
        kGravity * Math.pow(portRange, 2) * (1 + Math.pow(Math.tan(angle), 2))
        / (2 * (kShooterHeight - portHeight + portRange * Math.tan(angle))));
  }
  
  /**
   * Returns the angle the power cell makes when it reaches the center of the
   * outer port.
   * @param angle      The hood angle in radians
   * @param portRange  The range of the outer port horizontally.
   * @param portHeight The height of the outer port in meters.
   * @return The angle (in radians) relative to the power port that the power cell theoretically should make upon
   * reaching the outer port.
   */
  public double outerPortContactAngle(double angle, double portRange, double portHeight) {
    /*
     * The function is
     *                      2(Y-y_0)
     * Alpha(theta)=arctan(―――――――――  - tan(theta) )
     *                         X
     */
    return Math.atan(2 * (portHeight - kShooterHeight) / portRange - Math.tan(angle));
  }

  /**
   * Returns the angle the power cell will make relative to the floor at a certain distance from the robot.
   * @param velocity The shooter velocity (in meters/sec)
   * @param angle The hood angle (in radians)
   * @param x The horizontal distance the power cell has traveled (in meters)
   * @return The angle of the power cell's trajectory (in radians)
   */
  public double getTrajectoryAngleAtTime(double velocity, double angle, double x) {
    /*
     * The function is
     *                                       gx*(1+tan^2(theta))
     * J(v, theta, x) = arctan(tan(theta) - ―――――――――――――――――――――― )
     *                                              v^2
     */
    return Math.atan(Math.tan(angle) - (kGravity * x * (1 + Math.pow(Math.tan(angle), 2))) / (Math.pow(velocity, 2)));
  }

  /**
   * Returns the angle and velocity at which the power cell has zero vertical velocity
   * when it goes through the outer port.
   * @param portRange Distance to the outer port (in meters)
   * @param portHeight Height of the outer port (in meters)
   * @return The TrajectoryConfiguration object with the calculated angle and velocity.
   */
  public TrajectoryConfiguration getVertexConfiguration(double portRange, double portHeight) {
    /*
     * The function is
     *                           2 * (Y - y_0)
     *  theta_3(X, Y) = arctan( ―――――――――――――――― )
     *                                 X
     */
    return trajectoryConfigurationFromAngle(Math.atan(2 * (portHeight - kShooterHeight) / portRange), portRange, portHeight);
  }
  
  public TrajectoryConfiguration getDoublePortConfiguration(double portRange, double portHeight) {
    /*
     * The function is
     *                          (X_outer + X_inner) * (y_0 - Y)
     *  theta_2(X, Y) = arctan( ――――――――――――――――――――――――――――――― )
     *                                X_outer * X_inner
     */
    double innerPortRange = portRange + kRelativePortDistance;
    return trajectoryConfigurationFromAngle(
        Math.atan((portRange + innerPortRange) * (kShooterHeight - portHeight) / 
          (portRange * innerPortRange)), portRange, portHeight);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lights.setLEDLevel(1);
    AdvancedTrackerInterface.AdvancedTargetData[] targets;
    targets = portTracker.getAdvancedTargets();
    if(targets.length > 0) {                //makes sure we have a viable target
      currentTarget = targets[0];     
      if(currentTarget.quality >= 50){      //filters targets for high quality
        if(mode == CLAMode.VELOCITY){
          azimuthCorrection = (azimuthTarget - currentTarget.azimuth)* accelConstant;
        }
        if(mode == CLAMode.POSITION){
          azimuthCorrection = azimuthTarget + currentTarget.azimuth;
        }
        targetTrajectory = getDoublePortConfiguration(currentTarget.range, currentTarget.elevation);
        shooterVelocity = targetTrajectory.velocity;
        hoodAngle = targetTrajectory.angle;
        shooter.setHoodAngle(hoodAngle);
        // shooter.setFlywheelSpeed(getFlywheelRotationRate(shooterVelocity));
        if(mode == CLAMode.VELOCITY){
          turret.setVelocity(azimuthCorrection);
        }
        if(mode == CLAMode.POSITION){
          turret.setPosition(azimuthCorrection);
        }
        trackerStatus = "Tracking";
      }
      else {
        trackerStatus = "Low Quality";
        if(mode == CLAMode.VELOCITY){
          azimuthCorrection = (azimuthTarget - currentTarget.azimuth)* accelConstant * 0.5;
          turret.setVelocity(azimuthCorrection);
        }
      }
    }
    else {
      trackerStatus = "Target Not Found";
      turret.disable();
    }
  }

  /**
   * Gets the flywheel rotation rate given a desired speed
   * @param speed Speed of power cell (in meters/sec)
   * @return Motor speed suitable for Shooter.setFlywheelSpeed (in radians/sec)
   */
  public double getFlywheelRotationRate(double speed) {
    return speed * 0.5 * (kPowerCellRadius+kFlywheelRadius);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    turret.disable();
    trackerStatus = "Disabled";
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (temporary)
      return Math.abs(azimuthTarget - currentTarget.azimuth) < azimuthThreshold;
    else
      return false;
  }

  public String GetTrackerStatus(){
    return trackerStatus;
  }
  
  public TrajectoryConfiguration trajectoryConfigurationFromAngle(double angle, double portRange, double portHeight) {
    return new TrajectoryConfiguration(angle, getVelocity(angle, portRange, portHeight));
  }

  class TrajectoryConfiguration {
    double velocity = 0;
    double angle = 0;

    public TrajectoryConfiguration(double angle, double velocity) {
      this.velocity = velocity;
      this.angle = angle;
    }
  }
}
