/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.commands.BlingControls;


public class BlingControls extends CommandBase {
  int minLEDsVolts;
  int maxLEDsVolts;
  int numberLEDsVolts;
  double max_volts;
  double min_volts;
  int minLEDsDriver;
  int numberLEDsDriver;
  int maxLEDsDriver;
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
    
    // These variables determine the minimum and maximum LED values that are used
    // First port is 0
    minLEDsVolts = 0;
    numberLEDsVolts = 6;
    maxLEDsVolts = minLEDsVolts + numberLEDsVolts - 1;
    max_volts = 12.5;
    min_volts = 8;
    minLEDsDriver = 8;
    numberLEDsDriver = 4;
    maxLEDsDriver = minLEDsDriver + numberLEDsDriver - 1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    batteryBling();
    driverControlledLEDs();
  }

  public void batteryBling() {
    double volts = RobotController.getBatteryVoltage();

    // First, it calculates the percentage of leds that will turn on.
    // amount above the minimum voltage / range of volts
    // the -1 and +1 account for the one that is always on.
    int num = (int) (Math.round(((volts - min_volts) / (max_volts - min_volts)) * (numberLEDsVolts - 1)) + 1);

    // If less than 1/3 of the leds are lit up, the light is red.
    // If between 1/3 and 2/3 of the leds are lit up, the light is yellow.
    // If more than 2/3 of the leds are lit up, the light is green.
    if (num <= (numberLEDsVolts / 3)) {
      Robot.bling.rangeRGB(minLEDsVolts, num, 255, 0, 0);
    } else if (num > (numberLEDsVolts / 3) && num <= (2 * (numberLEDsVolts / 3))) {
      Robot.bling.rangeRGB(minLEDsVolts, num, 255, 255, 0);
    } else if (num > (2 * (numberLEDsVolts / 3))) {
      Robot.bling.rangeRGB(minLEDsVolts, num, 0, 255, 0);
    }
  }


  public void driverControlledLEDs(){
    if (OI.driverController.getStartButtonPressed()){
      // If start was pressed
      // set color
      Robot.bling.rangeRGB(minLEDsDriver, numberLEDsDriver, 255, 42, 0);
    } else if (OI.driverController.getBButtonPressed()){
      // If b was pressed
      // set colors to have alternating orange and blue
      Robot.bling.alternateRGB(minLEDsDriver, numberLEDsDriver, 255, 0, 0, 0, 0, 255);
    } else if (OI.driverController.getXButtonPressed()){
      // If x was pressed
      // set color
      Robot.bling.rangeRGB(minLEDsDriver, numberLEDsDriver, 0, 0, 255);
    } else if (OI.driverController.getYButtonPressed()){
      // If y was pressed
      // turn the light off
      Robot.bling.rangeRGB(minLEDsDriver, numberLEDsDriver, 0, 0, 0);
    }
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
