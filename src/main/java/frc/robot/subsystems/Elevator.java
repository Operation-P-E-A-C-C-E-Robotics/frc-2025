// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.TimedRobot;

/*The Elevatoré
 * Two falcon 500s 
 * no available srx on the robot atm, but the can id placeholder is 20 (which we swap out with the sushi's srx can id)
 */
public class Elevator extends SubsystemBase {
  //Uses CANcoders PARKER
  
  private static Joystick driverController = new Joystick(0);
  private double upSpeed = 10;
  private double downSpeed = -10;
  private int activeButton = 3; //x button on xbox
  private int deactiveButton = 4; //y button on xbox
  // private static Falcon;
  public static TalonSRX rightMotor = new TalonSRX(0); //falcon 500
  public static TalonSRX leftMotor = new TalonSRX(0);  
  //Todo    CHANGE THESE IDS WHEN THEIR DONE, and FIX THE RIGHT/LEFTOUT OUTPUT KEYBINDS
  // private final DutyCycleOut leftOut = new DutyCycleOut(0);
  // private final DutyCycleOut rightOut = new DutyCycleOut(0);
  
  /** Creates a new Elevator. */
  public Elevator()
  {
    
  }


  public void periodic() 
  {
    // rightOut.Output = driverController.getYChannel(); 
    // leftOut.Output = driverController.getYChannel();
    if(driverController.getRawButton(activeButton))
    {
      rightMotor.setInverted(false);
      leftMotor.setInverted(false);
      rightMotor.set(ControlMode.PercentOutput, upSpeed);
      leftMotor.set(ControlMode.PercentOutput, upSpeed);   
    }
    else if(driverController.getRawButton(deactiveButton))
    {      
      rightMotor.setInverted(true);
      leftMotor.setInverted(true);
      rightMotor.set(ControlMode.PercentOutput, downSpeed);
      leftMotor.set(ControlMode.PercentOutput, downSpeed);  

    }

  }
}
