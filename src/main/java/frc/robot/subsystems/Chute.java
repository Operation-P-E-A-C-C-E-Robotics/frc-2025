// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

public class Chute extends SubsystemBase {
  /** Creates a new Chute. */
  //2 BAG motors (1 talon fx) + a solenoid or servo, and we should set up (commented out) code for the possible second one, using master-slave system
  
  public static TalonFX rightChuteMotor;

  public Chute()
  {
     
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
