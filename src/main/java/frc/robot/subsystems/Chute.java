// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.motorcontrol.can.TalonSRX;
// import com.ctre.phoenix6.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.*;

public class Chute extends SubsystemBase {
  /** Creates a new Chute. */
  //2 BAG motors (1 talon srx) + a solenoid or servo, and we should set up (commented out) code for the possible second one, using master-slave system
  
  public static TalonSRX rightChuteMotor = new TalonSRX(28);
  public static TalonSRX leftChuteMotor = new TalonSRX(20);
  public boolean chutePressed = false;
  public double placeholderSpeed = 10;

  public void JFK(BooleanSupplier buttonID)
  {
    chutePressed = buttonID.getAsBoolean();
    System.out.println("JFKLIPPEDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD");
    System.out.println("JFKLIPPEDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD");    System.out.println("JFKLIPPEDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD");    System.out.println("JFKLIPPEDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD");    System.out.println("JFKLIPPEDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD");    System.out.println("JFKLIPPEDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD");    System.out.println("JFKLIPPEDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD");    System.out.println("JFKLIPPEDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD");    System.out.println("JFKLIPPEDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD");    System.out.println("JFKLIPPEDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD");    System.out.println("JFKLIPPEDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD");    System.out.println("JFKLIPPEDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD");
  }

  @Override
  public void periodic() 
  {
    if(chutePressed)
    {
      System.out.println("Recieving INPUT 1");
      rightChuteMotor.set(placeholderSpeed);
      System.out.println("Recieving INPUT 2");
      leftChuteMotor.setPosition(rightChuteMotor.getPosition().getValueAsDouble()); //ask shawne how to actually code master-slave motors
      System.out.println("Recieving INPUT 3");
    } 
    else
    {
      rightChuteMotor.stopMotor();
    }
  }
}
