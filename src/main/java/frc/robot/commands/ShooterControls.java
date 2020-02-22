/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  double pow = 0.5;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // speed = 0;
    // shooter.setFlywheelSpeed(speed);

    // double input = (0.5 * (shooter.getMinHoodAngle() + shooter.getMaxHoodAngle())
    //     + 0.5 * (shooter.getMaxHoodAngle() - shooter.getMinHoodAngle()) * OI.driverController.getRawAxis(0));
    // if (OI.driverController.getAButtonPressed()) {
    //   pow = 0.5 / pow;
    // }

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
    
    
    shooter.setHoodAngle((shooter.getMaxHoodAngle() + shooter.getMinHoodAngle()) * 0.5
      + (shooter.getMaxHoodAngle() - shooter.getMinHoodAngle()) * 0.5 * OI.driverController.getRawAxis(1));

    // shooter.setFlywheelPower(0.2 * pow * OI.driverController.getRawAxis(1));

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
