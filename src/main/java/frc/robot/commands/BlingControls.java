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
  int done;
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
    done = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (done == 0) {
      burst(Robot.bling.m_ledBuffer.getLength());
      Robot.bling.setPatternRGBAll(0, 0, 0);
    }
    if (done == 1) {
      blinkyLightsTwoColors();
    }
    //batteryBling(0, 6, 8, 12.5);
    // driverControlledLEDs(8, 4);
    // blinkyLights(14, 3);
    // movingLEDs(19, 7);
    
}

  public int burst(int length) {
    int middle1;
    int middle2;
    int i1 = 0;
    int time2 = 0;
    
    middle1 = (int) (Math.floor((length / 2)));
    middle2 = (int) (Math.ceil((length / 2)));

    i1 = middle1;

    for (int i2 = middle2; i2 <= length;) {
      if (time2 < 1000) {
        time2 = time2 + 1;
      } else {
        time2 = 0;
        i1 = i1 - 1;
        i2 = i2 + 1;
        Robot.bling.setPatternRGBAll(0, 0, 0);
      }
      Robot.bling.setLED(i1, 255, 255, 255);
      Robot.bling.setLED(i2, 255, 255, 255);
    }

    Robot.bling.setPatternRGBAll(0, 0, 0);
    done = 1;
    return done;
  }

  public void blinkyLightsTwoColors() {
    int time2 = 0;
    if (time2 < 50) {
      Robot.bling.setPatternHSVAll(240, 50, 50);
      time2 = time2 + 1;
    } else if (time2 < 100) {
      Robot.bling.setPatternRGBAll(255, 42, 0);
      time2 = time2 + 1;
    } else {
      time2 = 0;
    }
  }


  
  public void blinkyLights(int minLEDsBlink, int numberLEDsBlink) {
    int time = 0;
    if (time < 20) {
      Robot.bling.rangeRGB(minLEDsBlink, numberLEDsBlink, 0, 0, 0);
      time = time + 1;
    } else if (time < 40) {
      time = time + 1;
      Robot.bling.rangeRGB(minLEDsBlink, numberLEDsBlink, 255, 255, 255);
    } else {
      time = 0;
    }
  }

  public void batteryBling(int minLEDsVolts, int numberLEDsVolts, double min_volts, double max_volts) {
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

  public void movingLEDs(int minLEDsMove, int numberLEDsMove) {
    int move = 0;
    if (move < numberLEDsMove - 1) {
      move = move + 1;
    } else {
      move = 0;
    }
    int set = minLEDsMove + move;
    Robot.bling.rangeRGB(minLEDsMove, numberLEDsMove, 0, 0, 0);
    Robot.bling.m_ledBuffer.setRGB(set, 255, 0, 0);
  }


  public void driverControlledLEDs(int minLEDsDriver, int numberLEDsDriver){
    if (OI.driverController.getStartButtonPressed()){
      // If start was pressed
      // set color
      Robot.bling.rangeRGB(minLEDsDriver, numberLEDsDriver, 255, 42, 0);
    } else if (OI.driverController.getBButtonPressed()){
      // If b was pressed
      // set colors to have alternating orange and blue
      Robot.bling.alternateRGB(minLEDsDriver, numberLEDsDriver, 255, 42, 0, 0, 0, 255);
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
