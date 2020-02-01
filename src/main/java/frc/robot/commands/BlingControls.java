/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.commands.BlingControls;


public class BlingControls extends CommandBase {
  int burst_done;
  int burstCount;
  int time;
  int leds_from_middle;
  double match_time;

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
    burst_done = 0;
    time = 0;
    leds_from_middle = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    match_time = DriverStation.getInstance().getMatchTime();

    if (burst_done == 0) {
      burst(Robot.bling.m_ledBuffer.getLength());
      Robot.bling.setPatternRGBAll(0, 0, 0);
    }
    if (burst_done == 1) {
      
    /* if (match_time < 30) {
      blinkyLightsTwoColors(0, 255, 255, 0, 0, 0);
    } else {
      Robot.bling.setPatternRGBAll(0, 0, 0);
    } */
    }
    

     
    // driverControlledLEDs(8, 4);
    // blinkyLights(14, 3);
    // movingLEDs(19, 7);
  }

  public void GameData() {
    if (Robot.gameData.getGameData() returns "A")
  }


  public int burst(int length) {    
    // Calculates the middle led(s) of the led string
    int middle1 = (int) (Math.floor((length / 2)));
    int middle2 = (int) (Math.ceil((length / 2)));
    
    
    if ((leds_from_middle + middle2) <= length && time < 50) {
      // Wait until the 2000th time
      time = time + 1;
    } else if ((leds_from_middle + middle2) <= length) {
      time = 0;
      leds_from_middle = leds_from_middle + 1;
      Robot.bling.setPatternRGBAll(0, 0, 0);
      Robot.bling.setLEDs2(middle1 - leds_from_middle, middle2 + leds_from_middle, 0, 0, 255);
    } else {
      burst_done = 1;
      Robot.bling.setPatternRGBAll(0, 0, 0);
      time = 0;
    }

    Robot.bling.setLEDs2(middle1 - leds_from_middle, middle2 + leds_from_middle, 0, 0, 255);
    
    return burst_done;
  }


  public void blinkyLightsTwoColors(int h, int s, int v, int r, int g, int b) {
    if (time < 50) {
      Robot.bling.setPatternHSVAll(h, s, v);
      time = time + 1;
    } else if (time < 100) {
      Robot.bling.setPatternRGBAll(r, g, b);
      time = time + 1;
    } else {
      time = 0;
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
