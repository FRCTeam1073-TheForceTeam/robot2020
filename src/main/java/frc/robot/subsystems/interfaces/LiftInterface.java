/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.interfaces;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public interface LiftInterface {

    private static WPI_TalonSRX liftMotor;

    liftMotor = new WPI_TalonSRX(x);
    liftMotor.configFactoryDefault();
    liftMotor.setSafetyEnabled(false);
    liftMotor.setNeutralMode(NeutralMode.Brake);
    //liftMotor.configPeakOutputForward(1.0);
    //liftMotor.configPeakOutputReverse(-1.0);
    //liftMotor.setInverted(true);
    //liftMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    //liftMotor.setSensorPhase(true);

    }
