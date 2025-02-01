// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  //literally 2 falcons (with a setpoint and set angle distances from the setpoint, which the motor "counts by")
    // private final TalonFX tariyaki = new TalonFX(sushiMainID);
  private final TalonFX climbMaster = new TalonFX(10);
  private final MotionMagicExpoVoltage motionMagicControl = new MotionMagicExpoVoltage(0);
  //Your gonna want setposition to take in a "MotionMagicExpoVoltage"
  
  public Climber() {

  }

  public void setSpeed(double speed) {
    climbMaster.set(speed);
  }

  public void setPosition(double position) {
    climbMaster.setControl(motionMagicControl.withPosition(position));
  }
}