// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants.LEDConstants.ledMode;
import frc.robot.Constants.LEDConstants;



/** Add your docs here. */
public class LEDs extends SubsystemBase {
      /** Creates a new LED. */
private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LEDConstants.LEDlength);
private AddressableLED m_led = new AddressableLED(LEDConstants.LEDport);

private AddressableLEDBuffer red_ledBuffer = new AddressableLEDBuffer(LEDConstants.LEDlength);
private AddressableLEDBuffer blue_ledBuffer = new AddressableLEDBuffer(LEDConstants.LEDlength);
private AddressableLEDBuffer green_ledBuffer = new AddressableLEDBuffer(LEDConstants.LEDlength);
private AddressableLEDBuffer purple_ledBuffer = new AddressableLEDBuffer(LEDConstants.LEDlength);
private AddressableLEDBuffer yellow_ledBuffer = new AddressableLEDBuffer(LEDConstants.LEDlength);
private AddressableLEDBuffer team_ledBuffer = new AddressableLEDBuffer(LEDConstants.LEDlength);
private AddressableLEDBuffer rainbow_ledBuffer = new AddressableLEDBuffer(LEDConstants.LEDlength);

/*public enum ledMode {
  RED, GREEN, RAINBOW, TEAM, BLUE, PURPLE, YELLOW
}*/
  
//Constructor for LEDs class
public LEDs() {
  for (var i = 0; i < LEDConstants.LEDlength; i++) {
    red_ledBuffer.setRGB(i, 255, 0, 0);
    blue_ledBuffer.setRGB(i, 0, 0, 255);
    green_ledBuffer.setRGB(i, 0, 255, 0);
    purple_ledBuffer.setRGB(i, 135, 0, 211);
    yellow_ledBuffer.setRGB(i, 255, 255, 0);
  }

  //Set Team 1164 colors
  for (var i = 0; i < 6; i++) {
    team_ledBuffer.setRGB(i, 135, 0, 211);
    team_ledBuffer.setRGB(i+6, 255, 20, 0);
  }

  for (var i = 0; i < LEDConstants.LEDlength; i++) {
    final int hue = (int) (((((12 + i) * 180) / LEDConstants.LEDlength) + 0) % 180);
       rainbow_ledBuffer.setHSV(i, (hue), 255, 128);
  }

  setLEDmode(RobotContainer.LED_Chooser.getSelected());
  LED_init();
}

public void setLEDmode (LEDConstants.ledMode mode) {
  LEDConstants.ledMode m_mode = mode;

switch (m_mode) {
  case RED: m_ledBuffer = red_ledBuffer;
  break;
  case GREEN: m_ledBuffer = green_ledBuffer;
  break;
  case RAINBOW: m_ledBuffer = rainbow_ledBuffer;
  break;
  case TEAM: m_ledBuffer = team_ledBuffer;
  break;
  case BLUE: m_ledBuffer = blue_ledBuffer;
  break;
  case PURPLE: m_ledBuffer = purple_ledBuffer;
  break;
  case YELLOW: m_ledBuffer = yellow_ledBuffer;
  break;
  default:break;
}
}

public void setColor (LEDConstants.ledMode mode){ 
 setLEDmode(RobotContainer.LED_Chooser.getSelected());
 m_led.setData(m_ledBuffer);

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

private void setTEAM(int startPos, int Length) {
  for (var i = startPos; i < 8; i++) {
    m_ledBuffer.setRGB(i, 135, 0, 211);
  }
  for (var i = 8; i < startPos + Length; i++) {
    m_ledBuffer.setRGB(i, 255, 20, 0);
  }
}

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setColor(RobotContainer.LED_Chooser.getSelected());
  }
}
