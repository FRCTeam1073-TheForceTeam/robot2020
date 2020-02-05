/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.instances;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.GameDataInterface;

public class GameData extends SubsystemBase implements GameDataInterface {
    String gameData;
  /**
   * Creates a new Hook.
   */
  public GameData() {

  }

  public String getGameData() {
    // Recieves the color that the control panel needs to be set to
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    /*if(gameData.length() > 0) {
        switch (gameData.charAt(0)) {
        case 'B' :
          //Blue case code
          break;
        case 'G' :
          //Green case code
          break;
        case 'R' :
          //Red case code
          break;
        case 'Y' :
          //Yellow case code
          break;
        default :
          //This is corrupt data
          break;
        }
    } else {
        //Code for no data received yet
    } */
    return gameData;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
