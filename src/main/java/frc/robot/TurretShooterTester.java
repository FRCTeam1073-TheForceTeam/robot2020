/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ClosedLoopAiming;
import frc.robot.commands.DriveControls;
import frc.robot.commands.TurretControls;
import frc.robot.components.InterpolatorTable;
import frc.robot.subsystems.instances.*;
import frc.robot.subsystems.interfaces.AdvancedTrackerInterface;
import frc.robot.subsystems.interfaces.DrivetrainInterface;
import frc.robot.subsystems.interfaces.ShooterInterface;
import frc.robot.subsystems.interfaces.TurretInterface;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class TurretShooterTester extends TimedRobot {

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  
  // public static DriveControls command;
  // public static DrivetrainInterface subsystem;
  public static AdvancedTrackerInterface portTrackerCamera;
  public static CommandBase turretControls;
  public static TurretInterface turret;
  public static ShooterInterface shooter;
  // public NetworkTableEntry value_P;
  // public NetworkTableEntry value_I;
  // public NetworkTableEntry value_D;
  // public NetworkTableEntry update;

  // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
  @Override
  public void robotInit() {
    OI.init();
    portTrackerCamera = new OMVPortTracker(1);
    ((SubsystemBase) portTrackerCamera).register();
    turret = new Turret();
    ((SubsystemBase) turret).register();
    shooter = new Shooter();
    ((SubsystemBase) shooter).register();
    turretControls = new ClosedLoopAiming(turret, portTrackerCamera, shooter, false, 0.01);
    registerSubsystem((SubsystemBase) turret, turretControls);
  }

  public void registerSubsystem(SubsystemBase subsystem, CommandBase command) {
    subsystem.register();
    CommandScheduler.getInstance().setDefaultCommand(subsystem, command);
  }

  /*
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

  }

  @Override
  public void disabledPeriodic() {

  }
}
