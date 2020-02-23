/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.subsystems.interfaces.ShooterInterface;
import frc.robot.OI;

public class ShooterControls extends CommandBase {
  ShooterInterface shooter;
  double speed; 
  double flyVelocity = OI.operatorController.getRawAxis(1);
  double hoodVelocity = OI.operatorController.getRawAxis(5);

  public ShooterControls(ShooterInterface shooter_) {
    shooter = shooter_;
    addRequirements((SubsystemBase)shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  double pow = 0.0;
  double pow2 = 1;
  double value = 0;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // speed = 0;
    // shooter.setFlywheelSpeed(speed);
      // double input = (0.5 * (shooter.getMinHoodAngle() + shooter.getMaxHoodAngle())
      //     + 0.5 * (shooter.getMaxHoodAngle() - shooter.getMinHoodAngle()) * OI.driverController.getRawAxis(0));
    if (OI.driverController.getBumper(Hand.kLeft)) {
      pow = 0.0;
    } else if (OI.driverController.getAButtonPressed()) {
      pow = 0.25;
    }else if (OI.driverController.getBButtonPressed()) {
      pow = 0.5;
    } else if (OI.driverController.getXButtonPressed()) {
      pow = 0.75;
    } else if (OI.driverController.getYButtonPressed()) {
      pow = 1;
    }

    if (OI.driverController.getBumper(Hand.kRight)) {
      pow2 = 0.1;
    } else if (OI.driverController.getStartButtonPressed()) {
      pow2 = 1;
    }
    
    //Math.abs(OI.driverController.getRawAxis(1)) * Math.PI * 3.5;

    // double aput = OI.driverController.getRawAxis(1);
    // aput = Math.signum(aput) * Math.pow(Math.abs(aput), 0.65);
    // shooter.setHoodPower(0.05 * aput);

    // if (OI.driverController.getBumper(Hand.kLeft)) {
    //   shooter.setHoodPower(0.1*OI.driverController.getRawAxis(5));
    // }
    // if (OI.driverController.getBumperReleased(Hand.kLeft)) {
    //   shooter.resetHood();
    // }


    // shooter.setHoodPower(0.1 * OI.driverController.getRawAxis(1));
    
    
    // shooter.setHoodAngle((shooter.getMaxHoodAngle() + shooter.getMinHoodAngle()) * 0.5
    // + (shooter.getMaxHoodAngle() - shooter.getMinHoodAngle()) * 0.5 * OI.driverController.getRawAxis(1));

    value = pow2 * pow * OI.driverController.getRawAxis(1);
    shooter.setFlywheelSpeed(value * shooter.getMaximumFlywheelSpeed());
    SmartDashboard.putNumber("[Graph] Motor speed (RPM)", shooter.getFlywheelSpeed() * 60 / (2 * Math.PI));
    SmartDashboard.putNumber("[Graph] Estimated linear velocity of power cell (MPH)",
        shooter.getFlywheelSpeed() / (2.0 * Math.PI) * Math.PI * 0.5 * ((4.0 + 3.5) / 12.0) * (1.0 / 5280.0) * 3600.0);
    SmartDashboard.putNumber("[Value] Motor speed (RPM)", shooter.getFlywheelSpeed() * 60 / (2 * Math.PI));
    SmartDashboard.putNumber("[Value] Estimated linear velocity of power cell (MPH)",
        shooter.getFlywheelSpeed() / (2.0 * Math.PI) * Math.PI * 0.5 * ((4.0+3.5) / 12.0) * (1.0 / 5280.0) * 3600.0);
    SmartDashboard.putNumber("[Graph] TalonFX 22 motor temperature (degs. C)", shooter.getInternalTemperature()[0]);
    SmartDashboard.putNumber("[Graph] TalonFX 23 motor temperature (degs. C)", shooter.getInternalTemperature()[1]);
    SmartDashboard.putNumber("[Value] TalonFX 22 motor temperature (degs. C)", shooter.getInternalTemperature()[0]);
    SmartDashboard.putNumber("[Value] TalonFX 23 motor temperature (degs. C)", shooter.getInternalTemperature()[1]);
    SmartDashboard.putNumber("Percent multiplier", 100 * pow * pow2);
    SmartDashboard.putNumber("[Graph] Applied motor power", value);
    SmartDashboard.putNumber("[Value] Applied motor power", value);
    SmartDashboard.putNumber("[Graph] Battery voltage (Volts)", RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("[Value] Battery voltage (Volts)", RobotController.getBatteryVoltage());
    SmartDashboard.putString("Brownout status", RobotController.isBrownedOut() ? "BROWNED OUT" : "NOMINAL");

    // if (OI.driverController.getAButton()) {
    //   shooter.setHoodAngle(Math.PI * 1);
    // } else if (OI.driverController.getBButton()) {
    //   shooter.setHoodAngle(Math.PI * 2);      
    // } else if (OI.driverController.getXButton()) {
    //   shooter.setHoodAngle(Math.PI * 3);
    // } else if (OI.driverController.getYButton()) {
    //   shooter.setHoodAngle(Math.PI * 4);
    // } else {
    //   shooter.setHoodAngle(0);
    // }
    // shooter.setHoodAngle(OI.driverController.getRawAxis(1)*Math.PI*5);

    // System.out.println("HOOD: " + shooter.getHoodAngle() + ":" + shooter.getHoodVelocity());
    // System.out.println("TGT: " + input);

    // motor.set(0.25 * OI.driverController.getRawAxis(1));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}