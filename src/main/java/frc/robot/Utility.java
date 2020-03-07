package frc.robot;
public class Utility {
    /**
     * sets raw axis value inside the deadzone to zero
     * @param rawAxisValue
     * @return deadzoned axisValue
     */
    public static double deadzone(double rawAxisValue, double deadzone) {
        if (Math.abs(rawAxisValue) < deadzone) {
            return 0;
        } else {
            return (Math.signum(rawAxisValue) / (1 - deadzone)) * (Math.abs(rawAxisValue) - deadzone);
        }
    }

    public static double deadzone(double rawAxisValue) {
        return deadzone(rawAxisValue, Constants.CONTROLLER_DEADZONE);
    }
}