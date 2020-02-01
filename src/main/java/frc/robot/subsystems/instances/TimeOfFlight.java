package main.java.frc.robot.subsystems.instances;

public class TimeOfFlight extends SybsystemBase implements TimeOfFlightInterface{

    private static AnalogInput DistSensor;

    public TimeOfFlight(){

        DistSensor = new AnalogInput(0);//temporary port

    }
    @Override
    public static int convertToDistance(){
    }
    @Override
    public static int getSignal(){
        return DistSensor.getValue();
    }
    @Override
    public static void set(boolean isOn){
        
    }
}