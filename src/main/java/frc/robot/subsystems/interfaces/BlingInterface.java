package frc.robot.subsystems.interfaces;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public interface BlingInterface {
    public void setPatternRGBAll(int r, int g, int b);
    public void alternateRGB(int min, int number, int r1, int g1, int b1, int r2, int g2, int b2);
    public void rangeRGB(int min, int number, int r, int g, int b);
    public void setPatternHSVAll(int h, int s, int v);
    public void alternateHSV(int min, int number, int h1, int s1, int v1, int h2, int s2, int v2);
    public void rangeHSV(int min, int number, int h, int s, int v);
    public void setLED(int i, int r, int g, int b);
    public void setLEDs2(int i, int i2, int r, int g, int b);
    public AddressableLEDBuffer getM_LEDBuffer();
}