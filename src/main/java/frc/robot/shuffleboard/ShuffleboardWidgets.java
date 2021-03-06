/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.shuffleboard;

import java.util.Map;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.*;

/**The class - Defines all the variables used*/
public class ShuffleboardWidgets extends SubsystemBase {

  ShuffleboardTab tab;

  /*
   * private NetworkTableEntry P_testing; 
   * private NetworkTableEntry I_testing;
   * private NetworkTableEntry D_testing; 
   * double P_Value = 0.0; 
   * double I_Value = 0.0; 
   * double D_Value = 0.0;
   */

  private NetworkTableEntry leftEncoderEntry;
  double leftEncoderValue = 0.0;
  private NetworkTableEntry rightEncoderEntry;
  double rightEncoderValue = 0.0;
  private NetworkTableEntry drivetrainSpeedEntry;
  double drivetrainSpeed = 0.0;

  private NetworkTableEntry gyroAngleEntry;
  double gyroAngleDegrees = 0.0;
  private NetworkTableEntry xcoordinateEntry;
  double robotX = 0.0;
  private NetworkTableEntry ycoordinateEntry;
  double robotY = 0.0;
  private NetworkTableEntry rotationEntry;
  double robotRotation = 0.0;

  private NetworkTableEntry turretAngleEntry;
  double turretDegrees = 0.0;
  private NetworkTableEntry turretVelocityEntry;
  double turretVelocity = 0.0;

  private NetworkTableEntry flywheelVelocityEntry;
  double flywheelVelocity = 0.0;
  private NetworkTableEntry flywheelTemperature1Entry;
  private NetworkTableEntry flywheelTemperature2Entry;
  double[] flywheelTemperature = new double[2];
  private NetworkTableEntry hoodAngleEntry;
  double hoodDegrees = 0.0;
  private NetworkTableEntry hoodVelocityEntry;
  double hoodVelocity = 0.0;

  private NetworkTableEntry cellCountEntry;
  int cellCount = 0;

  private NetworkTableEntry isBrakesetEntry;
  boolean isBrakeset = false;
  private NetworkTableEntry isLiftFullyExtendedEntry;
  boolean isLiftFullyExtended = false;
  private NetworkTableEntry isLiftFullyRetractedEntry;
  boolean isLiftFullyRetracted = false;
  private NetworkTableEntry liftExtensionEntry;
  double liftExtension = 0.0;
  private NetworkTableEntry isPinnedEntry;
  boolean isPinned = false;

  private NetworkTableEntry isWinchEngagedEntry;
  boolean isWinchEngaged = false;

  private NetworkTableEntry batteryVoltageEntry;
  double batteryVoltage = 0.0;

  DrivetrainInterface drivetrain;
  TurretInterface turret;
  ShooterInterface shooter;
  MagazineInterface magazine;
  LiftInterface lift;
  WinchInterface winch;
  
  /**The constructor - creates the shuffleboard tab - gets the interfaces' methods to be able to display it - calls the method ShuffleboardView()*/
  public ShuffleboardWidgets(DrivetrainInterface drivetrain, TurretInterface turret, ShooterInterface shooter, MagazineInterface magazine, LiftInterface lift, WinchInterface winch) {

    //creates the tab in shuffleboard called Robot2020
    tab = Shuffleboard.getTab("Robot2020");

    this.drivetrain = drivetrain;
    this.turret = turret;
    this.shooter = shooter;
    this.magazine = magazine;
    this.lift = lift;
    this.winch = winch;

    /*
     * P_testing = tab .add("P", 1) .withWidget(BuiltInWidgets.kNumberSlider)
     * .withSize(10, 1) .withPosition(0, 1) .withProperties(Map.of("min", 0, "max",
     * 10)) .getEntry();
     * 
     * I_testing = tab .add("I", 0.01) .withWidget(BuiltInWidgets.kNumberSlider)
     * .withSize(10, 1) .withPosition(0, 2) .withProperties(Map.of("min", 0, "max",
     * 0.1)) .getEntry();
     * 
     * D_testing = tab .add("D", 10) .withWidget(BuiltInWidgets.kNumberSlider)
     * .withSize(10, 1) .withPosition(0, 3) .withProperties(Map.of("min", 0, "max",
     * 100)) .getEntry();
     */

    ShuffleboardWidgetInit();

  }

  /**used to update the values periodically*/
  @Override
  public void periodic() {

    ShuffleboardInformation();

    leftEncoderEntry.setDouble(leftEncoderValue);
    rightEncoderEntry.setDouble(rightEncoderValue);
    drivetrainSpeedEntry.setDouble(drivetrainSpeed);
    gyroAngleEntry.setDouble(gyroAngleDegrees);
    xcoordinateEntry.setDouble(robotX);
    ycoordinateEntry.setDouble(robotY);
    rotationEntry.setDouble(robotRotation);

    turretAngleEntry.setDouble(turretDegrees);
    turretVelocityEntry.setDouble(turretVelocity);

    flywheelVelocityEntry.setDouble(flywheelVelocity);
    flywheelTemperature1Entry.setDouble(flywheelTemperature[0]);
    flywheelTemperature2Entry.setDouble(flywheelTemperature[1]);

    hoodAngleEntry.setDouble(hoodDegrees);
    hoodVelocityEntry.setDouble(hoodVelocity);

    cellCountEntry.setDouble(cellCount);

    isBrakesetEntry.setBoolean(isBrakeset);
    isLiftFullyExtendedEntry.setBoolean(isLiftFullyExtended);
    isLiftFullyRetractedEntry.setBoolean(isLiftFullyRetracted);
    liftExtensionEntry.setDouble(liftExtension);
    isPinnedEntry.setBoolean(isPinned);
    isWinchEngagedEntry.setBoolean(isWinchEngaged);

    batteryVoltageEntry.setDouble(batteryVoltage);

    Shuffleboard.update();
    // PID_testing();
    
  }

  /**sets variables to data from the interfaces - gets called periodically*/
  private void ShuffleboardInformation() {

    // only pulls data from the drivetrain if it exists
    if(drivetrain != null){
      // Drivetrain Data COMPLETE
      leftEncoderValue = drivetrain.getLeftEncoder();
      rightEncoderValue = drivetrain.getRightEncoder();
      drivetrainSpeed = Math.sqrt(Math.pow(drivetrain.getDrivetrainVelocity().vxMetersPerSecond, 2) + Math.pow(drivetrain.getDrivetrainVelocity().vyMetersPerSecond, 2));
        // Gyro Datas COMPLETE
      gyroAngleDegrees = drivetrain.getAngleDegrees();
      Pose2d pose = drivetrain.getRobotPose();
      robotX = pose.getTranslation().getX();
      robotY = pose.getTranslation().getY();
      robotRotation = pose.getRotation().getDegrees();
    }

    // only pulls data from the turret if it exists
    if(turret != null){
      // Turret Data COMPLETE
      turretDegrees = turret.getPosition() * (1 / Math.PI) * 180;
      turretVelocity = turret.getVelocity() * (1 / Math.PI) * 180;
    }

    // only pulls data from the shooter if it exists
    if(shooter != null){
      // Shooter Data COMPLETE
      flywheelVelocity = shooter.getFlywheelSpeed() * (1 / Math.PI) * 180;
      flywheelTemperature = shooter.getInternalTemperature();
      hoodDegrees = shooter.getHoodAngle() * (1 / Math.PI) * 180;
      hoodVelocity = shooter.getHoodVelocity() * (1 / Math.PI) * 180;
    }

    // only pulls data from the magazine if it exists
    if(magazine != null){
      // Magazine Data COMPLETE
      cellCount = magazine.getCellCount();
    }

    // only pulls data from the lift if it exists
    if(lift != null){
      // Lift Data COMPLETE
      // isLiftFullyExtended = lift.isLiftFullyExtended();
      // isLiftFullyRetracted = lift.isLiftFullyRetracted();
      liftExtension = lift.liftExtension();
    }

    // only pulls data from the winch if it exists
    if(winch != null){
      // Winch Data Complete
      isWinchEngaged = winch.isWinchEngaged();
    }

    // Wheel of Fortune Data MISSING

    // General Robot
    batteryVoltage = RobotController.getBatteryVoltage();

  }

  /**Creates the widgets and sets them to their corresponding NetworkTableEntry */
  private void ShuffleboardWidgetInit() {

    leftEncoderEntry = tab
    .add("leftEncoder", leftEncoderValue)
    .withPosition(0, 0)
    .withSize(2, 1)
    .getEntry();
    rightEncoderEntry = tab
    .add("rightEncoder", rightEncoderValue)
    .withPosition(2, 0)
    .withSize(2, 1)
    .getEntry();
    drivetrainSpeedEntry = tab
    .add("drivetrainVelocity", drivetrainSpeed)
    .withPosition(0, 6)
    .withSize(2, 1)
    .getEntry();

    gyroAngleEntry = tab
    .add("gyroAngle", gyroAngleDegrees)
    .withPosition(2, 3)
    .withSize(2, 1)
    .getEntry();
    xcoordinateEntry = tab
    .add("x-coordinate", robotX)
    .withPosition(4, 0)
    .withSize(2, 1)
    .getEntry();
    ycoordinateEntry = tab
    .add("y-coordinate", robotY)
    .withPosition(6, 0)
    .withSize(2, 1)
    .getEntry();
    rotationEntry = tab
    .add("rotation", robotRotation)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -180, "max", 180))
    .withPosition(4, 3)
    .withSize(2, 2)
    .getEntry();

    turretAngleEntry = tab
    .add("turretDegrees", turretDegrees)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", 0, "max", 360))
    .withPosition(6, 3)
    .withSize(2, 2)
    .getEntry();
    turretVelocityEntry = tab
    .add("turretVelocity", turretVelocity)
    .withPosition(8, 0)
    .withSize(2, 1)
    .getEntry();

    flywheelVelocityEntry = tab
    .add("flywheelVelocity", flywheelVelocity)
    .withPosition(0, 1)
    .withSize(2, 1)
    .getEntry();
    flywheelTemperature1Entry = tab
    .add("flywheelTemperature1", flywheelTemperature[0] * (9 / 5) + 32)
    .withPosition(2, 1)
    .withSize(2, 1)
    .getEntry();
    flywheelTemperature2Entry = tab
    .add("flywheelTemperature2", flywheelTemperature[1] * (9 / 5) + 32)
    .withPosition(4, 1)
    .withSize(2, 1)
    .getEntry();
    hoodAngleEntry = tab
    .add("hoodDegrees", hoodDegrees)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", 0, "max", 180))
    .withPosition(8, 3)
    .withSize(2, 2)
    .getEntry();
    hoodVelocityEntry = tab
    .add("hoodVelocity", hoodVelocity)
    .withPosition(6, 1)
    .withSize(2, 1)
    .getEntry();

    cellCountEntry = tab
    .add("cellCount", cellCount)
    .withPosition(8, 1)
    .withSize(2, 1)
    .getEntry();

    isBrakesetEntry = tab
    .add("isBrakeset",isBrakeset)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withPosition(0, 2)
    .withSize(2, 1)
    .getEntry();
    isLiftFullyExtendedEntry = tab
    .add("isLiftFullyExtended",isLiftFullyExtended)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withPosition(2, 2)
    .withSize(2, 1)
    .getEntry();
    isLiftFullyRetractedEntry = tab
    .add("isLiftFullyRetracted",isLiftFullyRetracted)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withPosition(4, 2)
    .withSize(2, 1)
    .getEntry();
    liftExtensionEntry = tab
    .add("liftExtension",liftExtension)
    .withPosition(6, 2)
    .withSize(2, 1)
    .getEntry();
    isPinnedEntry = tab
    .add("isPinned",isPinned)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withPosition(8, 2)
    .withSize(2, 1)
    .getEntry();

    isWinchEngagedEntry = tab
    .add("isWinchEngaged",isWinchEngaged)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withPosition(0, 3)
    .withSize(2, 1)
    .getEntry();

    batteryVoltageEntry = tab
    .add("batteryVoltage", batteryVoltage)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", 0, "max", 14))
    .withPosition(0, 4)
    .withSize(2, 2)
    .getEntry();
      
  }

    /*public void PID_testing() {

      P_Value = P_testing.getDouble(1.0);
      I_Value = I_testing.getDouble(0.01);
      D_Value = D_testing.getDouble(10.0);

      //sets PID values as soon as PID tuning is complete the following line of code should be commented out until next year
      //drivetrain.setPID(P_Value, I_Value, D_Value);

      //prints the PID Values from Shuffleboard
      System.out.println(P_Value);
      System.out.println(I_Value);
      System.out.println(D_Value);

    }*/

}
