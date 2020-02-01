package main.java.frc.robot.subsystems.interfaces;

import edu.wpi.first.wpilibj.*;

public interface TimeOfFlight{

    int convertToDistance();
    //will convert the signal from Time-Of-Flight sensor to Distance value

    int getSignal();
    //return the raw signal from the Time-Of-Flight sensor

    void set(boolean isOn);
    //turns sensor on or off

}