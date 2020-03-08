/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.instances;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.MagazineInterface;

public class Magazine extends SubsystemBase implements MagazineInterface {
  /**
   * Creates a new Magazine.
   */
  private static int cellCount;// The cell count as determined by the trip of a distance sensor facing an opposite wall
  private static WPI_TalonSRX magMotor; //Motor controls all belts on magazine.
  // Will likely not have encoder.
  private static DigitalInput entrance;
  private static DigitalInput goingIn;
  private static DigitalInput goingOut;
  private static DigitalInput exit;
  private boolean cellEntering, cellIn, cellOut, cellExiting;

  public Magazine() {
    magMotor = new WPI_TalonSRX(26);
    cellCount = 4;
    // Initializes a four digital inputs with channels
    entrance = new DigitalInput(0);
    goingIn = new DigitalInput(1);
    goingOut = new DigitalInput(2);
    exit = new DigitalInput(3);

    magMotor.configFactoryDefault();
    magMotor.setSafetyEnabled(false);
    magMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // updateCellCount();
  }

  @Override
  /**
   * sets power to the magazine motor
   * 
   * @param - double power: between 0 and 1. The power to the motor
   */
  public void setPower(double power) {
    magMotor.set(ControlMode.PercentOutput, power);

  }

  @Override
  /**
   * updateCellCount() If a ball passes through the entrance, a power cell is
   * added The power cell number limit is 5. Param set to max of 6 in order to
   * provide  warning. Power cell value decreased after passing through exit.
   * 
   */
  public void updateCellCount() {

    if (entrance.get() == true && cellEntering == false && cellCount < 6) {
      cellCount++;
      cellEntering = true;
    }

    if (entrance.get() == false)
      cellEntering = false;
      

    // if (goingIn.get() == true && cellEntering == false && cellIn == false && cellCount < 6) {
    //   cellCount++;
    //   cellIn = true;
    // }

    // if (goingIn.get() == false)
    //   cellIn = false;


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
   * 
   * @return number of cells in the magazine
   */
  public int getCellCount() {
    return cellCount;
  }

  @Override
  public boolean getEnteranceState() {
    return entrance.get();
  }

  @Override
  public boolean getExitState() {
    return exit.get();
  }
}