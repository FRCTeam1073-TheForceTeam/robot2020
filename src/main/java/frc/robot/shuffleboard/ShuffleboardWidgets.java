/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.shuffleboard;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.interfaces.*;

public class ShuffleboardWidgets extends SubsystemBase {

  ShuffleboardTab tab;

  double leftEncoderValue = 0.0;
  double rightEncoderValue = 0.0;

  double gyroAngleDegrees = 0.0;
  double robotX = 0.0;
  double robotY = 0.0;
  double robotRotation = 0.0;

  double drivetrainVelocity = 0.0;

  private NetworkTableEntry P_testing;
  private NetworkTableEntry I_testing;
  private NetworkTableEntry D_testing;
  double P_Value;
  double I_Value;
  double D_Value;

  double turretDegrees = 0.0;
  double turretVelocity = 0.0;

  double flywheelVelocity = 0.0;
  double[] flywheelTemperature = new double[2];
  double hoodDegrees = 0.0;
  double hoodVelocity = 0.0;

  int cellCount = 0;

  boolean isBrakeset = false;
  boolean isLiftFullyExtended = false;
  boolean isLiftFullyRetracted = false;
  double liftExtension = 0.0;
  boolean isPinned = false;

  DrivetrainInterface drivetrain;
  TurretInterface turret;
  ShooterInterface shooter;
  MagazineInterface magazine;
  LiftInterface lift;

    public ShuffleboardWidgets() {

      tab = Shuffleboard.getTab("Telemetry");

      P_testing = tab
      .add("P", 1)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withSize(10, 1)
      .withPosition(0, 1)
      .withProperties(Map.of("min", 0, "max", 10))
      .getEntry();

      I_testing = tab
      .add("I", 0.01)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withSize(10, 1)
      .withPosition(0, 2)
      .withProperties(Map.of("min", 0, "max", 0.1))
      .getEntry();

      D_testing = tab
      .add("D", 10)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withSize(10, 1)
      .withPosition(0, 3)
      .withProperties(Map.of("min", 0, "max", 100))
      .getEntry();

      drivetrain = Robot.drivetrain;
      turret = Robot.turret;
      shooter = Robot.shooter;
      magazine = Robot.magazine;
      lift = Robot.lift;

      ShuffleboardView();

    }
  
  @Override
    public void periodic() {
      ShuffleboardInformation();
      Shuffleboard.update();
      PID_testing();

    }
  
    private void ShuffleboardInformation() {

      // Drivetrain Data INCOMPLETE
      leftEncoderValue = drivetrain.getLeftEncoder();
      rightEncoderValue = drivetrain.getRightEncoder();
        // Gyro Datas COMPLETE
      gyroAngleDegrees = drivetrain.getAngleDegrees();
      Pose2d pose = drivetrain.getRobotPose();
      robotX = pose.getTranslation().getX();
      robotY = pose.getTranslation().getY();
      robotRotation = pose.getRotation().getDegrees();

      // Turret Data INCOMPLETE?
      turretDegrees = turret.getPosition() * (1 / Math.PI) * 180;
      turretVelocity = turret.getVelocity() * (1 / Math.PI) * 180;

      // Shooter Data COMPLETE
      flywheelVelocity = shooter.getFlywheelSpeed() * (1 / Math.PI) * 180;
      flywheelTemperature = shooter.getInternalTemperature();
      hoodDegrees = shooter.getHoodAngle() * (1 / Math.PI) * 180;
      hoodVelocity = shooter.getHoodVelocity() * (1 / Math.PI) * 180; 

      // Magazine Data COMPLETE
      cellCount = magazine.getCellCount();

      // Climbing Data COMPLETE
      isBrakeset = lift.isBrakeSet();
      isLiftFullyExtended = lift.isLiftFullyExtended();
      isLiftFullyRetracted = lift.isLiftFullyRetracted();
      liftExtension = lift.liftExtension();
      isPinned = lift.isPinned();

      // Wheel of Fortune Data MISSING

    } 

    private void ShuffleboardView() {

      tab.add("leftEncoder", leftEncoderValue);
      tab.add("rightEncoder", rightEncoderValue);
      
      tab.add("gyroAngle", gyroAngleDegrees);
      tab.add("x-coordinate",robotX);
      tab.add("y-coordinate",robotY);
      tab.add("rotation",robotRotation);

      tab.add("turretDegrees", turretDegrees);
      tab.add("turretVelocity", turretVelocity);

      tab.add("flywheelVelocity",flywheelVelocity);
      tab.add("flywheelTemperature1", flywheelTemperature[0] * (9 / 5) + 32);
      tab.add("flywheelTemperature2", flywheelTemperature[1] * (9 / 5) + 32);
      tab.add("hoodDegrees",hoodDegrees);
      tab.add("hoodVelocity",hoodVelocity);

      tab.add("cellCount",cellCount);

      tab.add("isBrakeset",isBrakeset);
      tab.add("isLiftFullyExtended",isLiftFullyExtended);
      tab.add("isLiftFullyRetracted",isLiftFullyRetracted);
      tab.add("liftExtension",liftExtension);
      tab.add("isPinned",isPinned);
      
    }

    public void PID_testing() {

      P_Value = P_testing.getDouble(1.0);
      I_Value = I_testing.getDouble(0.01);
      D_Value = D_testing.getDouble(10.0);

      //sets PID values as soon as PID tuning is complete the following line of code should be commented out until next year
      //drivetrain.setPID(P_Value, I_Value, D_Value);

      //prints the PID Values from Shuffleboard
      System.out.println(P_Value);
      System.out.println(I_Value);
      System.out.println(D_Value);

    }

}
