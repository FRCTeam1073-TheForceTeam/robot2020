/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.autoCommands.*;
import frc.robot.commands.*;
import frc.robot.subsystems.instances.*;
import frc.robot.subsystems.instances.DrivetrainMercury;
import frc.robot.subsystems.interfaces.*;
import frc.robot.shuffleboard.*;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class RobotMercury extends TimedRobot {

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  public static DriveControls driveControls;
  public static DrivetrainInterface drivetrain;
  public static CollectorControls collectorControls;
  public static CollectorInterface collector;
  public static HookControls hookControls;
  public static HookInterface hook;
  public static LiftControls liftControls;
  public static LiftInterface lift;
  public static MagazineControls magazineControls;
  public static MagazineInterface magazine;
  public static ShooterControls shooterControls;
  public static Shooter shooter;
  public static TurretControls turretControls;
  public static TurretInterface turret;
  public static ShuffleboardWidgets widgets;

  public static BlingB autoBlingB;
  public static BlingA autoBlingA;
  public static Bling bling;
  public static BlingControls blingControls;
  public static SendableChooser<Command> chooser;
  public static autoDriveForward AutoDriveForward;
  public static WinchInterface winch;
  
  // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
  @Override
  public void robotInit() {
    // chooser = new SendableChooser<Command>();

    OI.init();

    drivetrain = new DrivetrainMercury();
    driveControls = new DriveControls(drivetrain, (WinchInterface)drivetrain);
    registerSubsystem((SubsystemBase) drivetrain, driveControls);
    
    // registerSubsystem((SubsystemBase) bling, blingControls);

    //autoBlingA = new BlingA(drivetrain, bling);

    //widgets = new ShuffleboardWidgets(drivetrain, turret, shooter, magazine, lift, (WinchInterface)drivetrain);
    //widgets.register();

    chooser.setDefaultOption("Unicorn Breath", new BlingA(bling));
    chooser.addOption("Medium Blue", new BlingB(bling));
    SmartDashboard.putData("Autonomous Mode", chooser);
  
    // autoBlingB = new BlingB(bling);
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
   */
  @Override
  public void autonomousInit() {
    if(autoBlingB != null){
      autoBlingB.schedule();
    }
    /* if(chooser.getSelected() != null){
     if(chooser.getSelected() != null){
      chooser.getSelected().schedule();
    }
    */
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
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
}
