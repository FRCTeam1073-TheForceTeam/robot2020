package frc.robot.subsystems.interfaces;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public interface ControlPanelInterface {
    /**
     * Engages control panel.
     */
    public void engageControlPanel();

    /**
     * Sets control panel velocity.
     * @param velocity The velocity to be set in radians/second.
     */
    public void setControlPanelVelocity(double velocity);

    /**
     * Gets control panel velocity.
     * @return The velocity in radians/second.
     */
    public double getControlPanelVelocity();
}