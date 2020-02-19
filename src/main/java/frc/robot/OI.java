package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class OI {
    public static XboxController driverController;
    public static XboxController operatorController;
    public static void init() {
        driverController = new XboxController(0);
        operatorController = new XboxController(1);
    }   
}