/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.smartdashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Robot;
/**
 * Add your docs here.
 */
public class DashboardInformation extends SubsystemBase {
    double leftEncoderValue;
    double rightEncoderValue;

    public DashboardInformation() {

    }
  
    @Override
    public void periodic() {
      sensorInputs();
    }
  
    public void sensorInputs() {
      //rightEncoderValue = Robot.subsystem.getRightEncoderValue();
      //leftEncoderValue = Robot.subsystem.getLeftEncoderValue();
    }
    /*  public double getLeftEncoderValue(){
      return left.getSelectedSensorPosition();
    }
  
    public double getRightEncoderValue(){
      return right.getSelectedSensorPosition();
    } */
  

}
