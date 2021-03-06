/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.subsystems.instances.*;
import frc.robot.subsystems.interfaces.*;
import frc.robot.shuffleboard.*;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class GearboxTester extends TimedRobot {

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  // public WPI_TalonFX leftLeader = new WPI_TalonFX(22);
  // public WPI_TalonFX leftFollower = new WPI_TalonFX(23);

  // public Solenoid solenoid1 = new Solenoid(1, 3);
  // public Solenoid solenoid2 = new Solenoid(1, 4);
  public CANSparkMax hood;
  
  // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
  @Override
  public void robotInit() {
    OI.init();
    hood=new CANSparkMax(25, MotorType.kBrushless);
    System.out.println(hood.getLastError());
    // leftFollower.follow(leftLeader);
    // leftFollower.setInverted(true);

    // solenoid1.set(true);
    // solenoid2.set(false);
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
    // JTJ - commenting out as we merge to master. re-enable with driveAuto is complete. 
    //if(driveAuto != null){
    //  driveAuto.schedule();
    //}
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // JTJ - commenting out as we merge to master. re-enable with driveAuto is complete. 
    //CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // leftLeader.set(ControlMode.PercentOutput,
    //     0.5 * (OI.driverController.getRawAxis(2) + 1) * OI.driverController.getRawAxis(1));

    // if (OI.driverController.getAButtonPressed()) {
    //   solenoid1.set(true);
    //   solenoid2.set(false);
    // } else if (OI.driverController.getBButtonPressed()) {
    //   solenoid1.set(false);
    //   solenoid2.set(true);
    // }
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