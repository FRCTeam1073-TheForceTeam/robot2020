/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class BlingControls extends CommandBase {
  
  /**
   * Creates a new BlingControls.
   */
  public BlingControls() {
    addRequirements(Robot.bling);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double volts = RobotController.getBatteryVoltage();

    /*
    if (11.5 < volts) {
      Robot.bling.numberRGB(8, 0, 255, 0);
    } else if (11 < volts && 11.5 > volts) {
      Robot.bling.numberRGB(7, 0, 255, 0);
    } else if (10.5 < volts && 11 > volts) {
      Robot.bling.numberRGB(6, 0, 255, 0);
    } else if (10 < volts && 10.5 > volts) {
      Robot.bling.numberRGB(5, 255, 255, 0);
    } else if (9.5 < volts && 10 > volts) {
      Robot.bling.numberRGB(4, 255, 255, 0);
    } else if (9 < volts && 9.5 > volts) {
      Robot.bling.numberRGB(3, 255, 255, 0);
    } else if (8.5 < volts && 9 > volts) {
      Robot.bling.numberRGB(2, 255, 0, 0);
    } else if (8 < volts && 8.5 > volts) {
      Robot.bling.numberRGB(1, 255, 0, 0);
    } else if (8 > volts) {
      Robot.bling.setPatternRGBAll(255, 0, 0);
    }
    */

    // Between 13 and 8

    int num = (int) (1 + Math.round(((volts - 8) / 5) * (8 /* m_ledBuffer.getLength() */ - 1)));
    Robot.bling.numberRGB(num, 255, 0, 0);

    /*
    //System.out.println("BlingControls.");
    if (OI.driverController.getStartButtonPressed()){
      // If a was pressed
      // set color red
      Robot.bling.setPatternRGBAll(255, 42, 0);
    } else if (OI.driverController.getBButtonPressed()){
      // If b was pressed
      // set color teal
      Robot.bling.alternateRGB(255, 0, 0, 0, 0, 255);
    } else if (OI.driverController.getXButtonPressed()){
      // If x was pressed
      // set color blue
      Robot.bling.setPatternRGBAll(0, 0, 255);
    } else if (OI.driverController.getYButtonPressed()){
      // If y was pressed
      // turn the light off
      Robot.bling.setPatternRGBAll(0, 0, 0);
    }
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
