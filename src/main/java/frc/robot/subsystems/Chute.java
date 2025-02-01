// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Wrist.WristSetpoints;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import static frc.robot.Constants.Chute.*;

// import com.ctre.phoenix6.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Chute extends SubsystemBase{
  /** Creates a new Chute. */
  //2 BAG motors (1 talon srx), and we should set up (commented out) code for the possible second one, using master-slave system

  public static TalonSRX rightChuteMotor = new TalonSRX(28); //Should be 28
  public static TalonSRX leftChuteMotor = new TalonSRX(16); //Should be 16

  public Chute(){  
    Trigger intakeButton = new JoystickButton(RobotContainer.commandController, intakeID); 
    intakeButton.onTrue(intake());
    Trigger dejamButton = new JoystickButton(RobotContainer.commandController, unjamID); 
    intakeButton.onTrue(unjam());
    // Invert the motor output if necessary
    rightChuteMotor.setInverted(false); 
    rightChuteMotor.configPeakCurrentLimit(40);

    // Invert the motor output if necessary
    leftChuteMotor.setInverted(false); 
    leftChuteMotor.configPeakCurrentLimit(40);
  }
  
  public void setSpeed(double leftSpeed, double rightSpeed) {
    leftChuteMotor.set(ControlMode.PercentOutput, leftSpeed);
    rightChuteMotor.set(ControlMode.PercentOutput, rightSpeed);
  }
  public Command intake()//will need to decide later which direction to run them in
  {
    return this.runOnce(() -> setSpeed(intakeSpeed, intakeSpeed));
  }

  public Command unjam()
  {
    return this.runOnce(() -> setSpeed(unjamSpeed, unjamSpeed));
  }
}
