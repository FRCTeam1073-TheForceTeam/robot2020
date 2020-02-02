/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.instances;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.MagazineInterface;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

public class Magazine extends SubsystemBase implements MagazineInterface {
  /** 
   * Creates a new Magazine.
   */
  private static int cellCount;
  private static WPI_TalonSRX magMotor;
  private static final int ballDist = 2;
  private static TimeOfFlight enterance;
  private static TimeOfFlight exit;
  private static TimeOfFlight height;

  public Magazine() {
    magMotor = new WPI_TalonSRX(24);
    enterance = new TimeOfFlight(1);
    exit = new TimeOfFlight(2);
    height = new TimeOfFlight(3);

    enterance.setRangingMode(RangingMode.Short, 20);
    exit.setRangingMode(RangingMode.Short, 20);
    height.setRangingMode(RangingMode.Short, 20);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getCellCount();
    getEnteranceDist();
    getExitDist();
  }

  @Override
  public static void run(double speed) {
    magMotor.set(speed);
  }
  @Override
  public static int getCellCount() {
    return cellCount;
  }
  @Over
  public static void getEnteranceDist(){
      double enterDist = enterance.getRange();
      if(enterDist <= 2.0){
          cellCount++;
      }
  }
  @Override
  public static void getExitDist(){
    double exitDist = exit.getRange();
    if(exitDist <= 2.0){
        cellCount--;
    }
  }
  

  
}
