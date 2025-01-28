// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import static frc.robot.Constants.Sushi.*;

public class Sushi extends SubsystemBase {

  private final TalonFX teriyaki = new TalonFX(20);
  private final StatusSignal<Angle> positionSignal;
  private final DutyCycleOut dutyCycle = new DutyCycleOut(0);
  private final PositionVoltage positionVoltage = new PositionVoltage(0);
  //get current motor pos and add difference to voltage thingy


  /** Creates a new Sushi. */
  //Set up literally one falcon 500, that (theoretically) triggers when you press a button/trigger. You can do this!!!
  // Backwards AND forwards
  public Sushi() {
    positionSignal = teriyaki.getPosition();
     BaseStatusSignal.setUpdateFrequencyForAll(100, 
      positionSignal,
      teriyaki.getDutyCycle(), 
      teriyaki.getVelocity(), 
      teriyaki.getAcceleration(), 
      teriyaki.getClosedLoopError(),
      teriyaki.getClosedLoopReference());
  }

  public void setSpeed(double speed) {
    teriyaki.set(speed);
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

  public void adjustCoral(double meters) {
    double motorRotations = metersToMotorRotations(meters);
    double currentPosition = positionSignal.getValueAsDouble();

    positionVoltage.withPosition(currentPosition + motorRotations);
    teriyaki.setControl(positionVoltage);
  }

  // Helper method to convert meters to ticks
  private double metersToMotorRotations(double meters) {

      // Meters -> Revolutions
      return (meters / wheelCircumference) * gearRatio;
  }
}

