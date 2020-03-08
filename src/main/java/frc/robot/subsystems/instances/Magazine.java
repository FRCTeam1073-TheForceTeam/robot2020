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
  private static DigitalInput exit;
  private boolean cellCheck, cellEntering, cellExiting;  

  private double P = 0.0;
  private double I = 0.0;
  private double D = 0.0;
  private double F = 0.0;

  public Magazine() {
    magMotor = new WPI_TalonSRX(26);
    cellCount = 0;
    // Initializes a four digital inputs with channels
        entrance = new DigitalInput(0);
        goingIn = new DigitalInput(1);
        exit = new DigitalInput(2);

    magMotor.configFactoryDefault();
    magMotor.setSafetyEnabled(false);
    magMotor.setNeutralMode(NeutralMode.Brake);

    magMotor.config_kP(0, P);
    magMotor.config_kI(0, I);
    magMotor.config_kD(0, D);
    magMotor.config_kF(0, F);
  }

  /**
   * sets the magazine speed in meters per second of the conveyor belts
   * 
   * @param double speed in meters per second
   */
  @Override
  public void setVelocity(double speed) {
    System.out.println("SETTING MAG VELOCITY");
    speed = speed / (0.0254 * 2 * Math.PI);
    magMotor.set(ControlMode.Velocity, speed);
  }

    @Override
    /**
     * First prox sensor trip checks if the cell has passed through the collector.
     * Second prox sensor trip adds a power cell, but only if the first is tripped.
     * Third prox sensor trip subtracts a power cell, as it will be exiting through the turret. 
     */
    public void updateCellCount() {

        if (goingIn.get() == true) {
            this.setPower(0.5);
            cellCheck = true;
        }
        if (goingIn.get() == false)
            this.setPower(0);
            cellCheck = false;

        if (entrance.get() == true && cellEntering == false && cellCount < 6 && cellCheck == false) {
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

            //TODO:Link with Turret when autonomous

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


  public double getVelocity(){
    return magMotor.getSelectedSensorVelocity(0);
  }

  public double getPower(){
    return magMotor.getMotorOutputPercent();
  }

  @Override
  public boolean getEntranceState(){
    return entrance.get();
  }

  @Override
  public boolean getGoingIn(){
    return goingIn.get();
  }

  @Override
  public void setPower(double speed) {
    magMotor.set(ControlMode.PercentOutput, speed);
  }
}