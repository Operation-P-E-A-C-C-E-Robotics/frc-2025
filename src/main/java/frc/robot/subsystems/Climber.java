// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Climber.leadClimberMotorID;

import java.util.function.DoubleSupplier;

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
  
  private double deployHeight = 10;//TODO
  private final TalonFX climbMaster = new TalonFX(leadClimberMotorID);
  public boolean deployed;

  //Motor configs + current limit + inversion + PID constants
  //Set default 
  
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
  }

  public void setSpeed(double speed) {
    if(deployed)
    {
      climbMaster.setControl(dutyCycle.withOutput(speed));      
    }
  }

  public void setPosition(double position) {
    climbMaster.setControl(motionMagicControl.withPosition(position));
  }

  public double getPosition()
  {
    return positionSignal.getValueAsDouble();
  }
  
  public Command rest()
  {
    return this.run(() -> setSpeed(0));
  }

  public Command getToClimbPos()
  {
      if(getPosition() < (deployHeight - 0.01))
      setPosition(0); //TODO figure out where climb pos is
      return null;
  }

  public Command manualInput(DoubleSupplier speed){
    return  this.run(() -> setSpeed(speed.getAsDouble())); //TODO figure out this axis
  }

  public Command deploy() {
    // Call setPosition on the Climber instance
     //elevator not allowed to go up  <- TODO make this, by changing the actual inputs required to 
     //make it go up in some way, once the ele inputs are done. //TODO IE get rid of the if statement below 
    return this.run(() -> {
      if(deployReady()) {
        setPosition(deployHeight);
        deployed = true;
      }
    });
   }
}