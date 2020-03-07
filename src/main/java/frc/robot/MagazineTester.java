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
import frc.robot.commands.CollectorControls;
import frc.robot.commands.MagazineControls;
import frc.robot.shuffleboard.ShuffleboardWidgets;
import frc.robot.subsystems.instances.Collector;
import frc.robot.subsystems.instances.Magazine;
import frc.robot.subsystems.interfaces.CollectorInterface;
import frc.robot.subsystems.interfaces.MagazineInterface;
import frc.robot.subsystems.interfaces.WinchInterface;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class MagazineTester extends TimedRobot {

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */

    // public static DriveControls driveControls;
    // public static DrivetrainInterface drivetrain;
    public static CollectorControls collectorControls;
    public static CollectorInterface collector;
    // public static HookControls hookControls;
    // public static HookInterface hook;
    // public static LiftControls liftControls;
    // public static LiftInterface lift;
    public static MagazineControls magazineControls;
    public static MagazineInterface magazine;
    //public static ShooterControls shooterControls;
    //public static Shooter shooter;
    //public static TurretControls turretControls;
    //public static TurretInterface turret;
    public static ShuffleboardWidgets widgets;
    //public static Bling bling;
    //public static BlingControls blingControls;
    //public AutoDrive driveAuto;

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    
    @Override
    public void robotInit() {
        OI.init();
        // bling = new Bling();
        // blingControls = new BlingControls(bling);
        // registerSubsystem((SubsystemBase) bling, blingControls);

        // drivetrain = new Drivetrain();
        // driveControls = new DriveControls(drivetrain);
        // registerSubsystem((SubsystemBase) drivetrain, driveControls);

        collector = new Collector();
        collectorControls = new CollectorControls(collector);
        registerSubsystem((SubsystemBase) collector, collectorControls);

        // hook = new Hook();
        // hookControls = new HookControls(hook);
        // registerSubsystem((SubsystemBase) hook, hookControls);

        // lift = new Lift();
        // liftControls = new LiftControls(lift);
        // registerSubsystem((SubsystemBase) lift, liftControls);

        magazine = new Magazine();
        magazineControls = new MagazineControls(magazine);
        registerSubsystem((SubsystemBase) magazine, magazineControls);

        // shooter = new Shooter();
        // shooterControls = new ShooterControls(shooter);
        // registerSubsystem((SubsystemBase) shooter, shooterControls);

        // turret = new Turret();
        // turretControls = new TurretControls(turret);
        // registerSubsystem((SubsystemBase) turret, turretControls);

        widgets = new ShuffleboardWidgets(null, null, null, magazine, null, (WinchInterface) null);
        //Would normally be: (drivetrain, turret, shooter, magazine, lift, (WinchInterface) drivetrain);
        widgets.register();

        // driveAuto = new AutoDrive(drivetrain, bling, 0.5, 4);
    }

    public void registerSubsystem(SubsystemBase subsystem, CommandBase command) {
        subsystem.register();
        CommandScheduler.getInstance().setDefaultCommand(subsystem, command);
    }

    /*
     * This function is called every robot packet, no matter the mode. Use this for
     * items like diagnostics that you would like to run during disabled, autonomous,
     * teleoperated and test modes.
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
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        // JTJ - commenting out as we merge to master. re-enable with driveAuto is
        // complete.
        // if(driveAuto != null){
        // driveAuto.schedule();
        // }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        // JTJ - commenting out as we merge to master. re-enable with driveAuto is
        // complete.
        // CommandScheduler.getInstance().run();
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