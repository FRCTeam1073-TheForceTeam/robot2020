/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.instances;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.MagazineInterface;

public class Magazine extends SubsystemBase implements MagazineInterface {
  /** 
   * Creates a new Magazine.
   */
  private static int cellCount;
  private static WPI_TalonSRX magMotor;
  private static Ultrasonic entrance;
  private static Ultrasonic exit;
  private static Ultrasonic height;

  public Magazine() {
    
    magMotor = new WPI_TalonSRX(24);
    entrance = new Ultrasonic(1, 2);//temp. parameters
    exit = new Ultrasonic(3, 4);//temp. parameters
    height = new Ultrasonic(5, 6);//temp. parameters

    enterance.setAutomaticMode(true);
    exit.setAutomaticMode(true);
    height.setAutomaticMode(true);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void run(double speed) {
    magMotor.set(speed);
  }

  @Override
  public int getCellCount() {
    return cellCount;
  }

  @Override
  public void addCell() {
    cellCount++;
  }

  @Override
  public void subtractCell() {
    cellCount--;
  }
}
