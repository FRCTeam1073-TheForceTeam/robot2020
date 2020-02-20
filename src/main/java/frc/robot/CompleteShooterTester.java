/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.*;

import frc.robot.subsystems.interfaces.*;
import frc.robot.subsystems.instances.*;
import frc.robot.commands.*;
import frc.robot.OI;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class CompleteShooterTester extends TimedRobot {


  public static OpenMVBase portTrackerCamera;
  public static TurretInterface turret;
  public static TurretIndex turretIndex;
  public static TurretControls turretControls;
  public static ShooterInterface shooter;
  public static ShooterIndex shooterIndex;
  public static ShooterControls shooterControls;
  public static OI oi;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    OI.init();
    // OpenMVBase camera = new OpenMVBase(1);
    OpenMVBase portTrackerCamera = new OMVPortTracker(1);
    portTrackerCamera.register();
    
    turret = new Turret();
    turretControls = new TurretControls(turret);
    registerSubsystem((SubsystemBase)turret, turretControls);

    shooter = new Shooter();
    shooterControls = new ShooterControls(shooter);
    registerSubsystem((SubsystemBase)shooter, shooterControls);

    System.out.println("CompleteShooterTester Init");
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

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   * This initializes the turret and shooter by returning whether or not they are indexed
   */
  @Override
  public void autonomousInit() {
    turretIndex = new TurretIndex(turret);
    if (turretIndex != null) {
      turretIndex.schedule();
    }
    // shooterIndex = new ShooterIndex();
    // if (shooterIndex != null) {
    //   shooterIndex.schedule();
    // }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (turretIndex != null) {
      turretIndex.cancel();
    }
     // if (shooterIndex != null) {
    //   shooterIndex.cancel();
    // }
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
}
