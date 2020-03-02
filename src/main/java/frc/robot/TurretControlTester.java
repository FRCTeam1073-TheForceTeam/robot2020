/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.instances.OpenMVBase;
import frc.robot.subsystems.instances.Shooter;
import frc.robot.commands.PointTurret;
import frc.robot.commands.ShooterControls;
import frc.robot.commands.ShooterIndex;
import frc.robot.commands.TurretControls;
import frc.robot.commands.TurretIndex;
import frc.robot.commands.ClosedLoopAiming.CLAMode;
import frc.robot.subsystems.instances.Lighting;
import frc.robot.commands.ClosedLoopAiming;
import frc.robot.commands.LightingControls;
import frc.robot.subsystems.instances.OMVPortTracker;
import frc.robot.subsystems.instances.Turret;
import frc.robot.subsystems.interfaces.AdvancedTrackerInterface;
import frc.robot.OI;

// THIS WILL FAIL ULTIMATELY UNTIL ENCODER VALUES ARE FIGURED OUT

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class TurretControlTester extends TimedRobot {

  // public static OpenMVBase portTrackerCamera;
  public static Turret turret;
  public static SequentialCommandGroup turretGroup;
  public static OI oi;
  public static Lighting lights;
  public static Shooter shooter;
  public static OpenMVBase portTrackerCamera;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    OI.init();
    portTrackerCamera = new OMVPortTracker(8);
    portTrackerCamera.register();
    turret = new Turret();
    turret.register();
    CommandScheduler.getInstance().setDefaultCommand((Subsystem) turret, new TurretControls(turret));
    lights = new Lighting(0);
    lights.register();
    // CommandScheduler.getInstance().setDefaultCommand((Subsystem) lights, new LightingControls(lights));
    shooter = new Shooter();
    shooter.register();
    CommandScheduler.getInstance().setDefaultCommand((Subsystem) shooter, new ShooterControls(shooter));

  }

  /*
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
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
    turret.disable();
    lights.setLEDLevel(0.0);
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
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
    turretGroup = new SequentialCommandGroup(new TurretIndex(turret), new PointTurret(turret, 0.0, 0.1));
    // , new ClosedLoopAiming(turret,
            // (AdvancedTrackerInterface) portTrackerCamera, shooter, lights, CLAMode.VELOCITY, false, 0.1));
    (new ShooterIndex(shooter)).schedule();
    // , new WaitForTurret(turret, 0.1, 0.1));
    if (turretGroup != null) {
      turretGroup.schedule();
    }
    if (lights != null){
      lights.setLEDLevel(1.0);
    }
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
