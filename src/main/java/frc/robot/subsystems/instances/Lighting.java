/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.instances;
import frc.robot.subsystems.interfaces.LightingInterface;
import edu.wpi.first.wpilibj.PWM;

/**
 * Controls the port-tracking lights
 */
public class Lighting extends OpenMVBase implements LightingInterface{
    private double LEDLevel = 0;
    private PWM ledPWM;

    public Lighting(int deviceId){
        super(deviceId);
        ledPWM = new PWM(0);
        ledPWM.setRaw(0);
        ledPWM.setPeriodMultiplier(PWM.PeriodMultiplier.k1X);
    }

    /**
     * gets brightness of the LEDs on the port-tracking OpenMV sensor module through RIO and PWM
     * @return double (0 - 1) 
     */
    @Override
    public double getLEDLevel(){
      return LEDLevel;
    }

    /**
     * sets brightness of the LEDs on the port-tracking OpenMV sensor module through RIO and PWM
     * set double (0 - 1)
     */
    @Override
    public void setLEDLevel(double illumLevel){
      System.out.println("LIGHTING SET "+illumLevel);
      if (illumLevel < 0){
        illumLevel = 0;
      }
      if (illumLevel > 1){
        illumLevel = 1;
      }
      LEDLevel = illumLevel;
      ledPWM.setRaw((int) (illumLevel * 2000));
    }
  
}