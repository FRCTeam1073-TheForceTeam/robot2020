package frc.robot.subsystems.instances;

//import com.ctre.phoenix.CANifier.LEDChannel;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.BlingInterface;

public class Bling extends SubsystemBase implements BlingInterface {
  public AddressableLED m_led;
  public AddressableLEDBuffer m_ledBuffer;
  public XboxController driverController;

  private int counter_rainbow = 0;
  private int move_rainbow = 0;

  public Bling() {
    m_led = new AddressableLED(4);
    m_ledBuffer = new AddressableLEDBuffer(29);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void setLEDData() {
    m_led.setData(m_ledBuffer);
  }

  // setPatternRGBAll sets the LEDs all to one color
  public void setPatternRGBAll(int r, int g, int b) {
    for (var i = 0; i < (m_ledBuffer.getLength()); i++) {
      m_ledBuffer.setRGB(i, r, g, b);
    }
    m_led.setData(m_ledBuffer);
  }

  // alternateRGB sets a range of LEDs where the even are one color and the odd are another
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

  // rangeRGB() sets a range of LEDs to one color
  public void rangeRGB(int min, int number, int r, int g, int b) {
    int max = min + number;
    for (int i = min; i < (max); i++) {
      m_ledBuffer.setRGB(i, r, g, b);
    }
    m_led.setData(m_ledBuffer);
  }

  // setPatternHSVAll() sets all of the LEDs to one color using HSV
  public void setPatternHSVAll(int h, int s, int v) {
    for (var i = 0; i < (m_ledBuffer.getLength()); i++) {
      m_ledBuffer.setHSV(i, h, s, v);
    }
    m_led.setData(m_ledBuffer);
  }

  // alternateHSV() has the same functionality as alternateRGB() except with HSV (hue, saturation + value)
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

  // rangeHSV() same as rangeRGB() except using HSV values
  public void rangeHSV(int min, int number, int h, int s, int v) {
    int max = min + number;

    for (int i = min; i < (max); i++) {
      m_ledBuffer.setHSV(i, h, s, v);
    }

    m_led.setData(m_ledBuffer);
  }

  public void setLED(int i, int r, int g, int b) {
    m_ledBuffer.setRGB(i, r, g, b);
    m_led.setData(m_ledBuffer);
  }

  // This sets two leds with the same color
  public void setLEDs2(int i, int i2, int r, int g, int b) {
    m_ledBuffer.setRGB(i, r, g, b);
    m_ledBuffer.setRGB(i2, r, g, b);
    m_led.setData(m_ledBuffer);
  }
  public AddressableLEDBuffer getM_LEDBuffer() {
    return m_ledBuffer;
  }

  public void LEDRainbow() {
    int numLEDs = m_ledBuffer.getLength();

    for (int i = 0; i < numLEDs; i++) {

      if (((i + move_rainbow) % 12) == 0 || ((i + move_rainbow) % 12) == 1) {
        // Sets first LED, then sets every 6 after it "red"
        m_ledBuffer.setRGB(i, 255, 0, 0);
      } else if (((i + 2 + move_rainbow) % 12) == 0 || ((i + 2 + move_rainbow) % 12) == 1) {
        // Sets second LED, then sets every 6 after it "orange"
        m_ledBuffer.setRGB(i, 255, 69, 0);
      } else if (((i + 4 + move_rainbow) % 12) == 0 || ((i + 4 + move_rainbow) % 12) == 1) {
        // Sets third LED, then sets every 6 after it "yellow"
        m_ledBuffer.setRGB(i, 252, 227, 0);
      } else if (((i + 6 + move_rainbow) % 12) == 0 || ((i + 6 + move_rainbow) % 12) == 1) {
        // Sets fourth LED, then sets every 6 after it "green"
        m_ledBuffer.setRGB(i, 0, 255, 0);
      } else if (((i + 8 + move_rainbow) % 12) == 0 || ((i + 8 + move_rainbow) % 12) == 1) {
        // Sets fifth LED, then sets every 6 after it "blue"
        m_ledBuffer.setRGB(i, 0, 0, 255);
      } else if (((i + 10 + move_rainbow) % 12) == 0 || ((i + 10 + move_rainbow) % 12) == 1) {
        // Sets sixth LED, then sets every 6 after it "purple"
        m_ledBuffer.setRGB(i, 128, 0, 128);
      }

      if (counter_rainbow < 100) {
        counter_rainbow++;
      } else {
        move_rainbow++;
        counter_rainbow = 1;
      }
      m_led.setData(m_ledBuffer);
    }
  }
}
