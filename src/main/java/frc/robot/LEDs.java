// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
//import frc.robot.Constants.LEDConstants.ledMode;
import frc.robot.Constants.LEDConstants;

/** Add your docs here. */
public class LEDs {
      /** Creates a new LED. */
private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LEDConstants.LEDlength);
private AddressableLED m_led = new AddressableLED(LEDConstants.LEDport);

/*public enum ledMode {
  RED, GREEN, RAINBOW, TEAM, BLUE, PURPLE, YELLOW
}*/
  
//Constructor for LEDs class
public LEDs() {
  setLEDmode(RobotContainer.LED_Chooser.getSelected());
  LED_init();
}

public void setLEDmode (LEDConstants.ledMode mode) {
  LEDConstants.ledMode m_mode = mode;

switch (m_mode) {
  case RED: setRED(0,LEDConstants.LEDlength);
  break;
  case GREEN: setGREEN(0, LEDConstants.LEDlength);
  break;
  case RAINBOW: rainbow(0, LEDConstants.LEDlength, 0, 0);
  break;
 // case TEAM: setTEAM(0, LEDConstants.LEDlength);
 //break;
  case BLUE: setBLUE(0, LEDConstants.LEDlength);
  break;
  case PURPLE: setPURPLE(0, LEDConstants.LEDlength);
  break;
  case YELLOW: setYELLOW(0, LEDConstants.LEDlength);
  break;
  default:break;
}
}

private void setRED(int startPos, int Length) {
  for (var i = startPos; i < startPos + Length; i++) {
    m_ledBuffer.setRGB(i, 255, 0, 0);
  }
}

private void setGREEN(int startPos, int Length) {
  for (var i = startPos; i < startPos + Length; i++) {
    m_ledBuffer.setRGB(i, 0, 255, 0);
  }
}

private void setBLUE(int startPos, int Length) {
  for (var i = startPos; i < startPos + Length; i++) {
    m_ledBuffer.setRGB(i, 0, 0, 255);
  }
}

private void setPURPLE(int startPos, int Length) {
  for (var i = startPos; i < startPos + Length; i++) {
    m_ledBuffer.setRGB(i, 135, 0, 211);
  }
}

private void setYELLOW(int startPos, int Length) {
  for (var i = startPos; i < startPos + Length; i++) {
    m_ledBuffer.setRGB(i, 255, 255, 0);
  }
}

/*
private void setTEAM(int startPos, int Length) {
  for (var i = startPos; i < 8; i++) {
    m_ledBuffer.setRGB(i, 135, 0, 211);
  }
  for (var i = 8; i < startPos + Length; i++) {
    m_ledBuffer.setRGB(i, 255, 20, 0);
  }
} */

  private void rainbow(int startPos, int Length, double rainbowOffset, double hueModdifier) {
    // if (startPos + Length < m_ledBuffer.getLength()) {
      for (var i = startPos; i < startPos + Length; i++) {
        final int hue = (int) (((((rainbowOffset + i) * 180) / Length) + hueModdifier) % 180);
        m_ledBuffer.setHSV(i, (hue), 255, 128);
      }
    }

  public void LED_init() {
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  } 

}
