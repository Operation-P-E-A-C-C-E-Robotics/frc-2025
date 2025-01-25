// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoderStickyFaults;

public class Sushi extends SubsystemBase {

  private double speed = 10; //In Percent
  private int forwardButtonID = 1;
  private int backwardButtonID = 2; //Would Require a Second Joystick Button IDK

  private final TalonSRX tariyaki = new TalonSRX(20);
  private static Joystick driverController = new Joystick(0);


  /** Creates a new Sushi. */
  //Set up literally one falcon 500, that (theoretically) triggers when you press a button/trigger. You can do this!!!
  // Backwards AND forwards
  public Sushi() {}

  public void periodic()
  {
    if(driverController.getRawButton(forwardButtonID)) {
      tariyaki.set(ControlMode.PercentOutput, speed);
    }
    else if(driverController.getRawButton(backwardButtonID)) {
      tariyaki.setInverted(true);
      tariyaki.set(ControlMode.PercentOutput, speed);
    }
    else
    {
      tariyaki.set(ControlMode.PercentOutput, 0);
    }
  }
}

