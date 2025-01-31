// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Chute extends SubsystemBase{
  /** Creates a new Chute. */
  //2 BAG motors (1 talon srx) + a solenoid or servo, and we should set up (commented out) code for the possible second one, using master-slave system
  public double placeholderSpeed = 10; //In Percent  //TODO i need sean's help to set up solenoid stuff 
  public int buttonID = 2;

  public static TalonSRX rightChuteMotor = new TalonSRX(28); //Should be 28
  public static TalonSRX leftChuteMotor = new TalonSRX(16); //Should be 16

  public void setSpeed(double leftSpeed, double rightSpeed)
  {

  } 
}