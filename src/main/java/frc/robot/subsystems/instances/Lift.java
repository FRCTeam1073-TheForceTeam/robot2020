/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.instances;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.LiftInterface;

public class Lift extends SubsystemBase implements LiftInterface {
  private static WPI_TalonSRX liftMotor;
  private static int min_encoder;
  private static int max_encoder;
  private int encoder_ticks;
  

  /**
   * Creates a new Lift.
   */
  
public Lift() {
    liftMotor = new WPI_TalonSRX(30);

    liftMotor.configFactoryDefault();
    liftMotor.enableCurrentLimit(true);
    liftMotor.configPeakCurrentLimit(35);
    liftMotor.configContinuousCurrentLimit(20);
    liftMotor.configPeakCurrentDuration(250);
    liftMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    liftMotor.configPeakOutputForward(1.0);
    liftMotor.setSelectedSensorPosition(0);
    
    // // TODO: add encoder limits
    // max_encoder = ___;
    // min_encoder = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    encoder_ticks = liftMotor.getSelectedSensorPosition();
  }

  public void liftExtend(double power) {
    // if (liftMotor.getSelectedSensorPosition() > min_encoder && liftMotor.getSelectedSensorPosition() < max_encoder){
      liftMotor.set(power);
    // }
  }


  public double liftExtension() {
    return liftMotor.getSelectedSensorPosition();
  }

  public int getEncoderTicks() {
    return encoder_ticks;
  }

  // public boolean isLiftFullyRetracted() {
  //   // Checks to see if the lift is fully retracted
    // if (liftMotor.getSelectedSensorPosition() <= min_encoder) {
  //     return true;
  //   } else {
  //     return false;
  //   }

  // }

  // public boolean isLiftFullyExtended() {
  //   // Checks to see if the lift is fully extended
    // if (liftMotor.getSelectedSensorPosition() >= max_encoder) {
  //     return true;
  //   } else {
  //     return false;
  //   }
    
  // }
  
}
