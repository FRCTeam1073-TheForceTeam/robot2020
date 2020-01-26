package frc.robot.subsystems.instances;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.BlingInterface;

public class Bling extends SubsystemBase implements BlingInterface {
  public AddressableLED m_led;
  public AddressableLEDBuffer m_ledBuffer;
  public XboxController driverController;

  public Bling() {
    m_led = new AddressableLED(7);
    m_ledBuffer = new AddressableLEDBuffer(29);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
    
  }

  public void setPatternRGBAll(int r, int g, int b) {
    for (var i = 0; i < (m_ledBuffer.getLength()); i++) {
      m_ledBuffer.setRGB(i, r, g, b);
    }
    m_led.setData(m_ledBuffer);
  }
  public void alternateRGB(int min, int number, int r1, int g1, int b1, int r2, int g2, int b2) {
    int max = min + number;
    for (int i = min; i < (max); i = i + 2) {
      m_ledBuffer.setRGB(i, r1, g1, b1);
    }
    for (int j = min + 1; j < (max); j = j + 2) {
      m_ledBuffer.setRGB(j, r2, g2, b2);
    }
    m_led.setData(m_ledBuffer);
  }
  public void rangeRGB(int min, int number, int r, int g, int b) {
    int max = min + number;
    for (int i = min; i < (max); i++) {
      m_ledBuffer.setRGB(i, r, g, b);
    }
    m_led.setData(m_ledBuffer);
  }

  public void setPatternHSVAll(int h, int s, int v) {
    for (var i = 0; i < (m_ledBuffer.getLength()); i++) {
      m_ledBuffer.setHSV(i, h, s, v);
    }
    m_led.setData(m_ledBuffer);
  }
  
  public void alternateHSV(int min, int number, int h1, int s1, int v1, int h2, int s2, int v2) {
    int max = min + number;
    for (int i = min; i < (max); i = i + 2) {
      m_ledBuffer.setHSV(i, h1, s1, v1);
    }
    for (int j = min + 1; j < (max); j = j + 2) {
      m_ledBuffer.setHSV(j, h2, s2, v2);
    }
    m_led.setData(m_ledBuffer);
  }
  public void rangeHSV(int min, int number, int h, int s, int v) {
    int max = min + number;
    for (int i = min; i < (max); i++) {
      m_ledBuffer.setHSV(i, h, s, v);
    }
    m_led.setData(m_ledBuffer);
  }
}