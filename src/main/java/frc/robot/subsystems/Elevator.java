// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.TimedRobot;

/*The Elevatoré
 * Two falcon 500s 
 * 
 */
public class Elevator extends SubsystemBase {
  //Uses CANcoders PARKER
  
  private static Joystick driverController = new Joystick(0);
  private double speed = 10;
  // private static Falcon;
  public static TalonFX rightMotor = new TalonFX(0); //falcon 500
  public static TalonFX leftMotor = new TalonFX(0);  
  //Todo    CHANGE THESE IDS WHEN THEIR DONE, and FIX THE RIGHT/LEFTOUT OUTPUT KEYBINDS
  private final DutyCycleOut leftOut = new DutyCycleOut(0);
  private final DutyCycleOut rightOut = new DutyCycleOut(0);
  
  /** Creates a new Elevator. */
  public Elevator()
  {
    
  }


  public void periodic() 
  {
    rightOut.Output = driverController.getYChannel(); 
    leftOut.Output = driverController.getYChannel();

    rightMotor.setControl(rightOut);
    leftMotor.setControl(leftOut);
  }
}
