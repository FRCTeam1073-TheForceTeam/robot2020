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
    m_led = new AddressableLED(9);
    m_ledBuffer = new AddressableLEDBuffer(8);
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
  public void alternateRGB(int r1, int g1, int b1, int r2, int g2, int b2) {
    for (int i = 0; i < (m_ledBuffer.getLength()); i = i + 2) {
      m_ledBuffer.setRGB(i, r1, g1, b1);
      
    }
    for (int j = 1; j < (m_ledBuffer.getLength());) {
      m_ledBuffer.setRGB(j, r2, g2, b2);
    j = j + 2;
    }
    m_led.setData(m_ledBuffer);
  }
  public void numberRGB(int number, int r, int g, int b) {
    for (int i = 0; i < (number); i++) {
      m_ledBuffer.setRGB(i, r, g, b);
    }
    m_led.setData(m_ledBuffer);
  }
}