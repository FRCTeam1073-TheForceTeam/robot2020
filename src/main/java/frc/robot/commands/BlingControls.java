/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.fasterxml.jackson.annotation.JsonSubTypes.Type;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.commands.BlingControls;
import frc.robot.subsystems.instances.Bling;
import frc.robot.subsystems.interfaces.BlingInterface;


public class BlingControls extends CommandBase {
  int burst_done;
  int burstCount;
  int time;
  int leds_from_middle;
  double match_time;
  int move;
  String gameData;
  BlingInterface bling;
  static Color red = new Color (255, 0, 0);
  static Color green = new Color (0, 255, 0);
  static Color blue = new Color (0, 0, 255);  

  /**
   * Creates a new BlingControls.
   */
  public BlingControls(BlingInterface bling_) {
    addRequirements((SubsystemBase)bling_);
    this.bling = bling_;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    burst_done = 0;
    time = 0;
    leds_from_middle = 0;
    move = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    bling.LEDRainbow();
    /* gameData = DriverStation.getInstance().getGameSpecificMessage();
    match_time = DriverStation.getInstance().getMatchTime();

    if (burst_done == 0) {
      burst(bling.getM_LEDBuffer().getLength());
      bling.setPatternRGBAll(0, 0, 0);
    } else {
      if (0 < match_time && match_time < 30) {
        blinkyLightsTwoColors(0, 255, 255, 0, 0, 0);
      } else {
        Robot.bling.setLEDFromColor(3);
        // driverControlledLEDs(8, 4);
        // blinkyLights(14, 3, 255, 255, 255);
        // movingLEDs(19, 7);
        bling.setPatternRGBAll(0, 0, 0);
      }
    }
    */
  }
  
  

//  public void GameData() {
//    if (Robot.gameData.getGameData() returns "A")
//  }

// burst() lights LEDs from the middle out  
public void burst(int length) {    
    // Calculates the middle led(s) of the led string
    int middle1 = (int) (Math.floor((length / 2)));
    int middle2 = (int) (Math.ceil((length / 2)));
    
    
    if ((leds_from_middle + middle2) < (length - 1) && time < 15) {
      // If there are still more LEDs to change and it is not yet time to change
      // Wait until the 2000th time
      time = time + 1;
    } else if ((leds_from_middle + middle2) < (length - 1)) {
      // If it is time to change and there are still more LEDs to change
      // Reset the time
      time = 0;
      // Moves the LEDs out from the center by one light
      leds_from_middle = leds_from_middle + 1;
      // Sets the LEDs
      bling.setPatternRGBAll(0, 0, 0);
      bling.setLEDs2(middle1 - leds_from_middle, middle2 + leds_from_middle, 0, 0, 255);
    } else {
      // Resets the time and says that the burst is finished
      burst_done = 1;
      time = 0;
    }
    bling.setLEDs2(middle1 - leds_from_middle, middle2 + leds_from_middle, 0, 0, 255);
  }


  // blinkyLightsTwoColors() switches the lights between two colors for all LEDs
  public void blinkyLightsTwoColors(int h, int s, int v, int r, int g, int b) {
    if (time < 50) {
      // Sets the LEDs to the first color
      bling.setPatternHSVAll(h, s, v);
      time = time + 1;
    } else if (time < 100) {
      // Sets the LEDs to the second color
      bling.setPatternRGBAll(r, g, b);
      time = time + 1;
    } else {
      // Resets the time
      time = 0;
    }
  }


  // blinkyLights() flashes lights on and off in one color for a range of LEDs
  public void blinkyLights(int minLEDsBlink, int numberLEDsBlink, int r, int g, int b) {
    int time = 0;
    if (time < 20) {
      // Turns the LEDs off
      bling.rangeRGB(minLEDsBlink, numberLEDsBlink, 0, 0, 0);
      time = time + 1;
    } else if (time < 40) {
      // Sets the LEDs to the specified color
      time = time + 1;
      bling.rangeRGB(minLEDsBlink, numberLEDsBlink, r, g, b);
    } else {
      // Resets the time counter
      time = 0;
    }
  }

  // batteryBling() sets the LED color and number depending on the battery voltage
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
      bling.rangeRGB(minLEDsVolts, num, 255, 0, 0);
    } else if (num > (numberLEDsVolts / 3) && num <= (2 * (numberLEDsVolts / 3))) {
      bling.rangeRGB(minLEDsVolts, num, 255, 255, 0);
    } else if (num > (2 * (numberLEDsVolts / 3))) {
      bling.rangeRGB(minLEDsVolts, num, 0, 255, 0);
    }
  }

  // movingLEDs() lights a single LED that moves up the range and then restarts
  public void movingLEDs(int minLEDsMove, int numberLEDsMove) {
    if (time < 50) {
      // Waits until the 50th time
      time = time + 1;
    } else {
      if (move < numberLEDsMove - 1) {
        // Changes the LED that is lit
        move = move + 1;
      } else {
        move = 0;
      }
      // Sets the LED that is lit
      int set = minLEDsMove + move;
      bling.rangeRGB(minLEDsMove, numberLEDsMove, 0, 0, 0);
      bling.setLED(set, 255, 0, 0);
    }
  }

  // driverControlledLEDs() allows the driver to control the LEDs using start, B, X, and Y
  public void driverControlledLEDs(int minLEDsDriver, int numberLEDsDriver){
    if (OI.driverController.getStartButtonPressed()){
      // If start was pressed
      // set color orange
      bling.rangeRGB(minLEDsDriver, numberLEDsDriver, 255, 42, 0);
    } else if (OI.driverController.getBButtonPressed()){
      // If b was pressed
      // set colors to have alternating orange and blue
      bling.alternateRGB(minLEDsDriver, numberLEDsDriver, 255, 42, 0, 0, 0, 255);
    } else if (OI.driverController.getXButtonPressed()){
      // If x was pressed
      // set color blue
      bling.rangeRGB(minLEDsDriver, numberLEDsDriver, 0, 0, 255);
    } else if (OI.driverController.getYButtonPressed()){
      // If y was pressed
      // turn the light off
      bling.rangeRGB(minLEDsDriver, numberLEDsDriver, 0, 0, 0);
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
