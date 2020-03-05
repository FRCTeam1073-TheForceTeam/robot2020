/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.commands.BlingControls;
import frc.robot.subsystems.interfaces.AdvancedTrackerInterface;
import frc.robot.subsystems.interfaces.BlingInterface;
import frc.robot.subsystems.interfaces.MagazineInterface;
import frc.robot.subsystems.interfaces.WinchInterface;


public class BlingControls extends CommandBase {
  int burst_done;
  int gameDataBlinkCount;
  int burstCount;
  int time;
  int time_burst;
  int time_blinkyLEDs;
  int leds_from_middle;
  double match_time;
  int move;
  String gameData;
  BlingInterface bling;
  WinchInterface winch;
  MagazineInterface magazine;
  int i_mag;
  AdvancedTrackerInterface portTracker;

  /**
   * Creates a new BlingControls.
   */
  public BlingControls(BlingInterface bling_, WinchInterface winch_, MagazineInterface magazine_,
      AdvancedTrackerInterface portTracker_) {
    addRequirements((SubsystemBase)bling_);
    this.bling = bling_;
    this.winch = winch_;
    this.magazine = magazine_;
    this.portTracker = portTracker_;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time_burst = 0;
    burst_done = 1;
    time = 0;
    time_blinkyLEDs = 0;
    leds_from_middle = 0;
    move = 0;
    gameDataBlinkCount = 0;
    i_mag = 0;
    SmartDashboard.putBoolean("Winch", winch.isWinchEngaged());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    match_time = DriverStation.getInstance().getMatchTime();

    if (burst_done == 0) {
      // burst(bling.getM_LEDBuffer().getLength(), 0, 0, 255);
      // bling.setPatternRGBAll(0, 0, 0);
    } else {
      if (gameData.equals("R") && gameDataBlinkCount < 5) {
        blinkyLights(0, bling.getM_LEDBuffer().getLength(), 255, 0, 0);

      } else if (gameData.equals("G") && gameDataBlinkCount < 5) {
        blinkyLights(0, bling.getM_LEDBuffer().getLength(), 0, 255, 0);

      } else if (gameData.equals("B") && gameDataBlinkCount < 5) {
        blinkyLights(0, bling.getM_LEDBuffer().getLength(), 0, 0, 255);

      } else if (gameData.equals("Y") && gameDataBlinkCount < 5) {
        blinkyLights(0, bling.getM_LEDBuffer().getLength(), 252, 227, 0);

      } else {
        // TODO: Add other bling commands

        winchVSdrivetrain(0, 2); // Who will win?

        // Changes the number and color of LEDS 3-9 based on the battery voltage
        batteryBling(2, 6, 8, 12.5);

        // Sets an LED for each ball in the collector
        magazineBallCountBling(9, magazine.getCellCount(), 255, 255, 0);

        // Sets LEDs based on the distance from the base of the power port
        powerCellTrackingBling(14, 10, 1.21, 8.53);
      }
    }
  }

  public void winchVSdrivetrain(int min_LEDs, int num_LEDs) {
    // The first two LEDs turn white if the winch is engaged
    if (winch.isWinchEngaged()) {
      bling.rangeRGB(min_LEDs, num_LEDs, 255, 255, 255);
    } else {
      bling.rangeRGB(min_LEDs, num_LEDs, 0, 0, 0);
    }
  }

  // burst() lights LEDs from the middle out  
  public void burst(int length, int r, int g, int b) {    
    // Calculates the middle led(s) of the led string
    int middle1 = (int) (Math.floor((length / 2)));
    int middle2 = (int) (Math.ceil((length / 2)));
    
    if ((leds_from_middle + middle2) < (length - 1) && time_burst < 15) {
      // If there are still more LEDs to change and it is not yet time to change
      // Wait until the 2000th time
      time_burst = time_burst + 1;
    } else if ((leds_from_middle + middle2) < (length - 1)) {
      // If it is time to change and there are still more LEDs to change
      // Reset the time
      time_burst = 0;
      // Moves the LEDs out from the center by one light
      leds_from_middle = leds_from_middle + 1;
      // Sets the LEDs
      bling.setPatternRGBAll(0, 0, 0);
      bling.setLEDs2(middle1 - leds_from_middle, middle2 + leds_from_middle, r, g, b);
    } else {
      // Resets the time and says that the burst is finished
      burst_done = 1;
      time_burst = 0;
      bling.setPatternRGBAll(0, 0, 0);
    }
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
    if (time_blinkyLEDs < 30) {
      // Sets the LEDs to the specified color
      bling.rangeRGB(minLEDsBlink, numberLEDsBlink, r, g, b);
      time_blinkyLEDs = time_blinkyLEDs + 1;
    } else if (time_blinkyLEDs < 60) {
      // Turns the LEDs off
      time_blinkyLEDs = time_blinkyLEDs + 1;
      bling.rangeRGB(minLEDsBlink, numberLEDsBlink, 0, 0, 0);
    } else {
      // Resets the time counter
      time_blinkyLEDs = 0;
      gameDataBlinkCount = gameDataBlinkCount + 1;
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

  public void magazineBallCountBling(int min_LEDs, int ballCount, int r, int g, int b) {
    bling.rangeRGB(min_LEDs, ballCount, r, g, b);
  }

  public void powerCellTrackingBling(int minLEDs, int numLEDs, double min_meters, double max_meters, int r, int g, int b) {
    if (portTracker.getAdvancedTargets()[0].quality > 0) {
      int num = (int) (Math.round(((portTracker.getAdvancedTargets()[0].range - min_meters) /
          (max_meters - min_meters)) * (numLEDs - 1)) + 1);
      bling.rangeRGB(minLEDs, num, r, g, b);
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
