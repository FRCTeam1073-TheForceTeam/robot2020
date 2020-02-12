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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.LiftInterface;

public class Lift extends SubsystemBase implements LiftInterface {
  
  WPI_TalonSRX liftWinch;
  private boolean hasLiftStopped = false;
  private double liftPower = 0.0;

  //the TalonSRX ID
  int x = 0;

  //some value for the liftWinch control
  double y = 0.0;

  public Lift() {

    liftWinch = new WPI_TalonSRX(x);

    liftWinch.configFactoryDefault();
    liftWinch.setSafetyEnabled(true);
    liftWinch.setNeutralMode(NeutralMode.Brake);
    //liftWinch.configPeakOutputForward(1.0);
    //liftWinch.configPeakOutputReverse(-1.0);
    //liftWinch.setInverted(true);
    //liftWinch.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    //liftWinch.setSensorPhase(true);

  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    if (liftPower == 0  && !hasLiftStopped) {
      hasLiftStopped = true;
    }
    if (liftPower != 0 && hasLiftStopped) {
      hasLiftStopped = false;
    }

  }
  
  public void setVelocity(double lift) {
  liftWinch.set(ControlMode.Velocity, lift*y);
  liftPower = lift;
  }

  public void setPower(double lift) {
  liftWinch.set(ControlMode.PercentOutput, lift);
  liftPower = lift;
  }

  public double getLiftEncoder() {
  return liftWinch.getSelectedSensorPosition();
  }

}
