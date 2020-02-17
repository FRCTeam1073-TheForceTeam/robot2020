/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.instances;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.MagazineInterface;

public class Magazine extends SubsystemBase implements MagazineInterface {
  /** 
   * Creates a new Magazine.
   */
  private static int cellCount; //cell count determined by enterance and exit counter
  private static int heightCellCount;//The cell count as determined by the distance sensor facing upward
  private static WPI_TalonSRX magMotor; //motor controls all belts on magazine. Will likely not have encoder
  private static final int ballDist = 2;
  //private static TimeOfFlight enterance;
  //private static TimeOfFlight exit;
  //private static TimeOfFlight height;//distance sensor facing upward from the bottom of the magazine

  public Magazine() {
    magMotor = new WPI_TalonSRX(24);
    cellCount = 0;
    //enterance = new TimeOfFlight(1);
    //exit = new TimeOfFlight(2);
    //height = new TimeOfFlight(3);

    //enterance.setRangingMode(RangingMode.Short, 20);
    //exit.setRangingMode(RangingMode.Short, 20);
    //height.setRangingMode(RangingMode.Short, 20);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    int cellCount = getCellCount();
    //int enterDist = updateEnteranceDist();
    //int exitDist = updateExitDist();
  }

  @Override
  public void run(double power) {
    magMotor.set(power);
  }
  @Override
  public int getCellCount() {
    return cellCount;
  }
  @Override
  public void updateExitDist(){
  }
  @Override
  public void updateHeightCellCount(){

  }
  @Override
  public void updateEnteranceDist() {

  }
  

  
}
