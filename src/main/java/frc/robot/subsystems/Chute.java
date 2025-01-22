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
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.RobotContainer;
public class Chute extends SubsystemBase {
  /** Creates a new Chute. */
  //2 BAG motors (1 talon srx) + a solenoid or servo, and we should set up (commented out) code for the possible second one, using master-slave system
  
  public static TalonSRX rightChuteMotor = new TalonSRX(28);
  public static TalonSRX leftChuteMotor = new TalonSRX(20);
  public boolean chutePressed = false;
  public double placeholderSpeed = 10;

  public void JFK(BooleanSupplier buttonID)
  {
    System.out.println("SomewhatWorking");
    System.out.println("SomewhatWorking");
    System.out.println("SomewhatWorking");
    System.out.println("SomewhatWorking");
    System.out.println("SomewhatWorking");
    System.out.println("SomewhatWorking");
    System.out.println("SomewhatWorking");
    System.out.println("SomewhatWorking");
    System.out.println("SomewhatWorking");
    System.out.println("SomewhatWorking");
    System.out.println("SomewhatWorking");
    System.out.println("SomewhatWorking");
    System.out.println("SomewhatWorking");
    System.out.println("SomewhatWorking");
    System.out.println("SomewhatWorking");
    System.out.println("SomewhatWorking");
    System.out.println("SomewhatWorking");
    System.out.println("SomewhatWorking");
    System.out.println("SomewhatWorking");
    System.out.println("SomewhatWorking");
    System.out.println("SomewhatWorking");
    System.out.println("SomewhatWorking");
    System.out.println("SomewhatWorking");
    System.out.println("SomewhatWorking");
    System.out.println("SomewhatWorking");
    System.out.println("SomewhatWorking");
    System.out.println("SomewhatWorking");
    System.out.println("SomewhatWorking");
    System.out.println("SomewhatWorking");
    System.out.println("SomewhatWorking");
    System.out.println("SomewhatWorking");
    System.out.println("SomewhatWorking");

    chutePressed = buttonID.getAsBoolean();
  }

  @Override
  public void periodic() 
  {
    if(RobotContainer.driverController.getRawButton(0))
    {
      System.out.println("Recieving INPUT 1");
      rightChuteMotor.set(ControlMode.PercentOutput, placeholderSpeed);
      System.out.println("Recieving INPUT 2");
      // leftChuteMotor.setPosition(rightChuteMotor.getPosition().getValueAsDouble()); //ask shawne how to actually code master-slave motors
      leftChuteMotor.set(ControlMode.PercentOutput, placeholderSpeed);
      System.out.println("Recieving INPUT 3");
    } 
    else
    {
      rightChuteMotor.set(ControlMode.PercentOutput, 0);
      leftChuteMotor.set(ControlMode.PercentOutput, 0);
      System.out.println("Recieving INPUT 4");
    }
  }
}
