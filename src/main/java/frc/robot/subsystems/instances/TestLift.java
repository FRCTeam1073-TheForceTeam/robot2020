/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.instances;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.LiftInterface;
import com.ctre.phoenix.motorcontrol.can.*;

public class TestLift extends SubsystemBase implements LiftInterface {

  WPI_TalonSRX liftLeader;
  WPI_TalonSRX liftFollower;

  /**
   * Creates a new TestLift.
   */
  public TestLift() {
    liftLeader = new WPI_TalonSRX(30/*whatever*/);
    liftFollower = new WPI_TalonSRX(31/*whatever*/);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public double liftExtension() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public void liftExtend(double power) {
    // TODO Auto-generated method stub

  }

  @Override
  public int getEncoderTicks() {
    // TODO Auto-generated method stub
    return 0;
  }
}
