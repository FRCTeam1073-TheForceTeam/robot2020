/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.TestCommand;
import frc.robot.subsystems.TestSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.*;
import java.util.HashMap;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public static TestCommand command;
  public static TestSubsystem subsystem;
  public NetworkTableEntry value_P;
  public NetworkTableEntry value_I;
  public NetworkTableEntry value_D;
  public NetworkTableEntry update;


  // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
  @Override
  public void robotInit() {
    RobotContainer.init();
    OI.init();
    subsystem = new TestSubsystem();
    command = new TestCommand(subsystem);
    System.out.println(command == null);
    System.out.println(subsystem == null);
    subsystem.register();
    CommandScheduler.getInstance().setDefaultCommand(subsystem, command);
    /*HashMap map=new HashMap<String,Object>();
    map.put("Min",0);
    map.put("Max",5);
    value_P = Shuffleboard.getTab("Drive")
                        .add("value_P", 0)
                        .withWidget(BuiltInWidgets.kNumberSlider)
                        .withSize(4,1)
                        .withProperties(map)
                        .getEntry();
    
    value_I = Shuffleboard.getTab("Drive")
                        .add("value_I", 0)
                        .withWidget(BuiltInWidgets.kNumberSlider)
                        .withSize(4,1)
                        .withProperties(map)
                        .getEntry();

    value_D = Shuffleboard.getTab("Drive")
                        .add("value_D", 0)
                        .withWidget(BuiltInWidgets.kNumberSlider)
                        .withSize(4,1)
                        .withProperties(map)
                        .getEntry();*/
    
    
    // update = Shuffleboard.getTab("Drive")
    //                     .add("update", 0)
    //                     .withWidget(BuiltInWidgets.kToggleSwitch)
    //                     .getEntry();
    
    

    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    //System.out.println(value_P.getDouble(-1));
    /*if (update.getBoolean(false)){
      SmartDashboard.putBoolean("update",false);
      Shuffleboard.getTab("Drive").add("[Testing]",0);
      System.out.println("Switch_off");
    }*/
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
    // // schedule the autonomous command (example)
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
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
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.cancel();
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
