/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.instances;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
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
  

  /**
   * Creates a new Lift.
   */
  // Solenoid solenoid = new Solenoid(7);
  
  // // that sets minimum possible value the potentiometer  can return (when the voltage from the potentiometer is 0 (the minimum))
  // private final double minLiftExtension = 0.0;
  // // that sets maximum possible value the potentiometer can return (when the voltage from the potentiometer is 1 (the maximum))
  // private final double maxLiftExtension = 1.0;

  // // these values have to be set to the measured physical limits using the potentiometer HAVE TO BE SET
  // private final double physicalMinExtension = 0.0;
  // private final double physicalMaxExtension = 10.0;
  
  // // defines the potentiometerPort HAS TO BE SET
  // private final int potentiometerPort = 0;

  // // Initializes an AnalogInput on port "potentiometerPort"
  // AnalogInput potentiometerValue = new AnalogInput(potentiometerPort);

  // // Initializes an AnalogPotentiometer on port "potentiometerValue"
  // AnalogPotentiometer potentiometer = new AnalogPotentiometer(potentiometerValue, minLiftExtension, maxLiftExtension);

  public Lift() {
    liftMotor = new WPI_TalonSRX(30);

    liftMotor.configFactoryDefault();
    liftMotor.enableCurrentLimit(true);
    liftMotor.configPeakCurrentLimit(35);
    liftMotor.configContinuousCurrentLimit(20);
    liftMotor.configPeakCurrentDuration(250);
    liftMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    liftMotor.configPeakOutputForward(1.0);
    
    // TODO: add encoder limits
    max_encoder = ___;
    min_encoder = ___;


    // Enables 2-bit averaging
    // potentiometerValue.setAverageBits(2);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void liftExtend(double power) {
    if (liftMotor.getSelectedSensorPosition() > min_encoder && liftMotor.getSelectedSensorPosition() < max_encoder){
      liftMotor.set(power);
    }
  }

  // public void setBrakeOn() {
  //   // Turns on the brake
  // }

  // public void setBrakeOff() {
  //   // Turns off the brake
  // }

  // public boolean isBrakeSet() {
  //   // Checks to see if the brake is set
  //   return true;
  // }

  // public double liftExtension() {

  //   // returns in the scale from minLiftExtention to maxLiftExtention
  //   return potentiometer.get();

  // }

  // public boolean isLiftFullyRetracted() {
  //   // Checks to see if the lift is fully retracted
  //   if (liftExtension() <= physicalMinExtension) {
  //     return true;
  //   } else {
  //     return false;
  //   }

  // }

  // public boolean isLiftFullyExtended() {
  //   // Checks to see if the lift is fully extended
  //   if (liftExtension() >= physicalMaxExtension) {
  //     return true;
  //   } else {
  //     return false;
  //   }
    
  // }
  
}
