// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Climber.leadClimberMotorID;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climber extends SubsystemBase {
  private final MotionMagicExpoVoltage motionMagicControl = new MotionMagicExpoVoltage(0);
  private final StatusSignal<Angle> positionSignal;
  private final DutyCycleOut dutyCycle = new DutyCycleOut(0);

  private final TalonFX climbMaster = new TalonFX(leadClimberMotorID);
  //Your gonna want setposition to take in a "MotionMagicExpoVoltage"
  
  public Climber() {
    positionSignal = climbMaster.getPosition();
    BaseStatusSignal.setUpdateFrequencyForAll(100,
      positionSignal,
      climbMaster.getDutyCycle(),
      climbMaster.getVelocity(),
      climbMaster.getAcceleration(),
      climbMaster.getClosedLoopError(),
      climbMaster.getClosedLoopReference()
    );
    rest();
  }

  public void setSpeed(double speed) {
    climbMaster.setControl(dutyCycle.withOutput(speed));
  }

  public void setPosition(double position) {
    climbMaster.setControl(motionMagicControl.withPosition(position));
  }

  public double getPosition()
  {
    return positionSignal.getValueAsDouble();
  }

  /** the climber does nothing */
  public Command rest()
  {
    return this.run(() -> setSpeed(0));
  }
}