/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.instances;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.MagazineInterface;

public class Magazine extends SubsystemBase implements MagazineInterface {
  /** 
   * Creates a new Magazine.
   */
  private static int cellCount;//The cell count as determined by the distance sensor facing upward
  private static WPI_TalonSRX magMotor; //motor controls all belts on magazine. Will likely not have encoder
  private static DigitalInput enterance;
  private static DigitalInput exit;
  



  public Magazine() {
    magMotor = new WPI_TalonSRX(24);//24 is temporary ID
    cellCount = 0;
    enterance = new DigitalInput(1);
    exit = new DigitalInput(2);
    

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
    magMotor.set(power);
  }
  @Override
  /**
   * updateCellCount()
   * if a ball passes through the enterance, a power cell is added
   * if a ball passes through the exit, a power cell is decreased
   */
  public void updateCellCount() {

    if(enterance.get() == true){
      cellCount++;
    }
    if(exit.get() == true){
      cellCount--;
    }

  }
  
  @Override
  /**
   * getCellCount()
   * @return number of cells in the magazine
   */
  public int getCellCount(){
    return cellCount;
  }
}