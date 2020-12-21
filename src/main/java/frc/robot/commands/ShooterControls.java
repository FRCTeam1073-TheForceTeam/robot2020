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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utility;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.interfaces.ShooterInterface;
import frc.robot.subsystems.instances.Shooter;

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
    shooter.resetHood();
  }

  double pow = 0.0;
  double pow2 = 1;
  double value = 0;
  double shooterGear = 0;
  final double maxGear = 7;
  double hoodAngleAccumulator = 0;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(OI.operatorController.getStartButton()){
    //   shooter.setFlywheelSpeed(-1.301 * (4000.0  * 2.0 * Math.PI) / 60.0);
    //   shooter.setHoodAngle((1.259 * 2.0 * Math.PI) / Shooter.kMotorRadiansPerHoodRadian + shooter.getMinHoodAngle());
    // }
    if(deadzone(OI.operatorController.getTriggerAxis(Hand.kRight)) == 0){
      if(OI.operatorController.getBumper(Hand.kLeft)) shooter.setFlywheelSpeed(0);
      else{
        if(OI.operatorController.getBumperPressed(Hand.kLeft)&&OI.operatorController.getStartButton()){
          shooterGear=Math.min(shooterGear+1,maxGear);
        }
        if(OI.operatorController.getBackButtonPressed()){
          shooterGear=Math.max(shooterGear-1,0);
        }
        if(OI.operatorController.getBackButtonPressed()&&OI.operatorController.getStartButton()){
          shooterGear=0;
        }
        speed = -1.301 * deadzone((shooterGear / maxGear) * 670.0);
        shooter.setFlywheelSpeed(speed);
      }
    }
    
    if(OI.operatorController.getBackButtonPressed()){
      SmartDashboard.putNumber("Input RPM Snapshot",  speed * 60 / (2 * Math.PI));
      SmartDashboard.putNumber("Output RPM Snapshot", shooter.getFlywheelSpeed() * 30.0 / Math.PI);
    }

    shooter.setDeadzoneRollerPower(OI.operatorController.getBumper(Hand.kRight) ? 0.75 : 0);

    
    // shooter.setDeadzoneRollerVelocity(OI.operatorController.getRawAxis(2) - OI.operatorController.getRawAxis(3));
    
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
    hoodAngleAccumulator += (deadzone(OI.operatorController.getRawAxis(5)) / 30.0);
    hoodAngleAccumulator = Utility.clip(hoodAngleAccumulator, -1, 1);
    
    shooter.setHoodAngle((shooter.getMaxHoodAngle() + shooter.getMinHoodAngle()) * 0.5
        + (shooter.getMaxHoodAngle() - shooter.getMinHoodAngle()) * 0.5 * hoodAngleAccumulator);

    // value = pow2 * pow * OI.operatorController.getRawAxis(1);
    // shooter.setFlywheelSpeed(value * shooter.getMaximumFlywheelSpeed());
    // SmartDashboard.putNumber("Input RPM", value * shooter.getMaximumFlywheelSpeed() * 30 / Math.PI);
    // SmartDashboard.putNumber("[Graph] Motor speed (RPM)", shooter.getFlywheelSpeed() * 60 / (2 * Math.PI));
    // SmartDashboard.putNumber("[Graph] Estimated linear velocity of power cell (MPH)",
        // shooter.getFlywheelSpeed() / (2.0 * Math.PI) * Math.PI * 0.5 * ((4.0 + 3.5) / 12.0) * (1.0 / 5280.0) * 3600.0);
    SmartDashboard.putNumber("[Value] Motor speed (RPM)", shooter.getFlywheelSpeed() * 60 / (2 * Math.PI));

    // SmartDashboard.putNumber("[Value] Estimated linear velocity of power cell (MPH)",
        // shooter.getFlywheelSpeed() / (2.0 * Math.PI) * Math.PI * 0.5 * ((4.0 + 3.5) / 12.0) * (1.0 / 5280.0) * 3600.0);
    // SmartDashboard.putNumber("Percent multiplier", 100 * pow * pow2);
    // SmartDashboard.putNumber("[Graph] Applied motor power", value);
    SmartDashboard.putNumber("[Value] Applied motor power", value);
    // SmartDashboard.putNumber("[Graph] Battery voltage (Volts)", RobotController.getBatteryVoltage());
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

  /**
     * sets raw axis value inside the deadzone to zero
     * @param rawAxisValue
     * @return deadzoned axisValue
     */
    public double deadzone(double rawAxisValue) {
      if (Math.abs(rawAxisValue) < Constants.CONTROLLER_DEADZONE){
          return 0;
      } else {
          return rawAxisValue;
      }
  }

}