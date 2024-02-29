// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants.LEDConstants.ledMode;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.ledMode;



/** Add your docs here. */
public class LEDs extends SubsystemBase {
      /** Creates a new LED. */
private AddressableLEDBuffer m_ledBuffer; 
private AddressableLED m_led;
private ledMode curr_color;
private boolean new_change;
private AddressableLEDBuffer red_ledBuffer;
private AddressableLEDBuffer blue_ledBuffer;
private AddressableLEDBuffer green_ledBuffer;
private AddressableLEDBuffer purple_ledBuffer;
private AddressableLEDBuffer yellow_ledBuffer;
private AddressableLEDBuffer team_ledBuffer;
private AddressableLEDBuffer rainbow_ledBuffer;

Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
/*public enum ledMode {
  RED, GREEN, RAINBOW, TEAM, BLUE, PURPLE, YELLOW
}*/
  
//Constructor for LEDs class
public LEDs(int length, int port) {
  new_change = false;
  curr_color = ledMode.PURPLE;
  m_ledBuffer = new AddressableLEDBuffer(length);
  m_led = new AddressableLED(port);
  red_ledBuffer = new AddressableLEDBuffer(length);
  blue_ledBuffer = new AddressableLEDBuffer(length);
  green_ledBuffer = new AddressableLEDBuffer(length);
  purple_ledBuffer = new AddressableLEDBuffer(length);
  yellow_ledBuffer = new AddressableLEDBuffer(length);
  team_ledBuffer = new AddressableLEDBuffer(length);
  rainbow_ledBuffer = new AddressableLEDBuffer(length);

  for (var i = 0; i < length; i++) {
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

  for (var i = 0; i < length; i++) {
    final int hue = (int) (((((12 + i) * 180) / length) + 0) % 180);
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
  case ALLIANCE: {
     if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Blue) {
              m_ledBuffer = blue_ledBuffer;
            } if (alliance.get() == Alliance.Red) {
              m_ledBuffer = red_ledBuffer;
            }
        }
      }
  default:break;
}
}

public void setColor (LEDConstants.ledMode mode) { 
  LEDConstants.ledMode m_led_mode = mode;
  setLEDmode(m_led_mode);
  m_led.setData(m_ledBuffer);
}

public void changeColor (ledMode colorChange){
  curr_color = colorChange;
  new_change = true;
}

  public void LED_init() {
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (new_change) {
      setColor(curr_color);
      new_change = false;
    }
  }
}
