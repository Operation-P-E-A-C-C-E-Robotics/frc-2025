// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;
import com.ctre.phoenix6.hardware.TalonFX;

public class Chute extends SubsystemBase {
  /** Creates a new Chute. */
  //2 BAG motors (1 talon fx) + a solenoid or servo, and we should set up (commented out) code for the possible second one, using master-slave system
  
  public static TalonFX rightChuteMotor = new TalonFX(77);
  public static TalonFX leftChuteMotor = new TalonFX(78);
  public boolean chutePressed = false;
  public double placeholderSpeed = 10;

  public void JFK(BooleanSupplier buttonID)
  {
    chutePressed = buttonID.getAsBoolean();
  }

  @Override
  public void periodic() 
  {
    if(chutePressed)
    {
      rightChuteMotor.set(placeholderSpeed);
      leftChuteMotor.setPosition(rightChuteMotor.getPosition().getValueAsDouble()); //ask shawne how to actually code master-slave motors
    } 
    else
    {
      rightChuteMotor.stopMotor();
    }
  }
}
