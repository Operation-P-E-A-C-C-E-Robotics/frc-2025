// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.Sushi.*;

public class Sushi extends SubsystemBase {

  private final TalonFX tariyaki = new TalonFX(0);
  private final StatusSignal<Angle> positionSignal;
  private final DutyCycleOut dutyCycle = new DutyCycleOut(0);
  //get current motor pos and add difference to voltage thingy


  /** Creates a new Sushi. */
  //Set up literally one falcon 500, that (theoretically) triggers when you press a button/trigger. You can do this!!!
  // Backwards AND forwards
  public Sushi() {

    tariyaki = new TalonFX(sushiMainID);


    ParentDevice.optimizeBusUtilizationForAll(tariyaki);
    positionSignal = tariyaki.getPosition();
     BaseStatusSignal.setUpdateFrequencyForAll(100, 
      positionSignal,
      tariyaki.getDutyCycle(), 
      tariyaki.getVelocity(), 
      tariyaki.getAcceleration(), 
      tariyaki.getClosedLoopError(),
      tariyaki.getClosedLoopReference());
  }

  public void setSpeed(double speed)
  {
    tariyaki.set(speed);
  }
  
  //Implement all of this in command
  // public void periodic() {
  //   if(driverController.getRawButton(forwardButtonID)) {
  //     setSpeed(defaultSpeed);
  //   }
  //   else if(driverController.getRawButton(backwardButtonID)) {
  //     setSpeed(-defaultSpeed);
  //   }
  //   else {
  //     setSpeed(0);
  //   }
  // }

  public void adjustCoral(double meters)
  {
    
  }

}

