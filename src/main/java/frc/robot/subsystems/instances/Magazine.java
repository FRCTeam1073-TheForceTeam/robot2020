/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.instances;

//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.MagazineInterface;

public class Magazine extends SubsystemBase implements MagazineInterface {
  /** 
   * Creates a new Magazine.
   */
  private static int cellCount;//The cell count as determined by the trip of a distance sensor facing an opposite wall
  //private static WPI_TalonSRX magMotor; //Motor controls all belts on magazine. Will likely not have encoder.
  private static DigitalInput entrance;
  private static DigitalInput exit;
  private boolean cellEntering, cellExiting;

  public Magazine() {
    //magMotor = new WPI_TalonSRX(26);//24 is temporary ID
    cellCount = 0;
//Initializes a digital input with channel
    entrance = new DigitalInput(0);
    exit = new DigitalInput(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateCellCount();
  }

  @Override
  /**
   * run
   * sets power to the magazine motor
   * @param - double power: between 0 and 1. The power to the motor
   */
  public void run(double power) {
    //magMotor.set(power);
    
  }

  @Override
  /**
   * updateCellCount()
   * If a ball passes through the entrance, a power cell is added
   * The power cell number limit is 5. Param set to max of 6 in order to provide a warning.
   * If a ball passes through the exit, a power cell is decreased
   * 
   */ 
  public void updateCellCount() {

    if (entrance.get() == true && cellEntering == false && cellCount < 6) {
      cellCount++;
      cellEntering = true;
    }
      
    if (entrance.get() == false)
      cellEntering = false;

    if (exit.get() == true && cellExiting == false && cellCount > 0) {
      cellCount--;
      cellExiting = true;
    }

    if (exit.get() == false)
      cellExiting = false;

    if (cellCount > 5)
      System.out.println("Jack - stop! You have more than 5 power cells.");

    SmartDashboard.putNumber("Cell Count", cellCount);
  }
  
  @Override
  /**
   * getCellCount()
   * @return number of cells in the magazine
   */
  public int getCellCount() {
    return cellCount;
  }
}
