// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import static frc.robot.Constants.Sushi.*;

import java.util.function.DoubleSupplier;

public class Sushi extends SubsystemBase {

  private final TalonFX tariyaki = new TalonFX(sushiMainID);
  private final StatusSignal<Angle> positionSignal;
  private final PositionVoltage positionVoltage = new PositionVoltage(0);

  private DigitalInput frontBeamBreak = new DigitalInput(frontBeamBreakID); //Todo: ask if shawne meant two *Sets* of beam blocks, 
  private DigitalInput backBeamBreak = new DigitalInput(backBeamBreakID); //or two *Induvidual* beam block sensors

  //get current motor pos and add difference to voltage thingy
//todo configure motor thingies the same way elevator did it

  /** Creates a new Sushi. */
  //Set up literally one falcon 500, that (theoretically) triggers when you press a button/trigger. You can do this!!!
  // Backwards AND forwards
  public Sushi() {

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

  public void setSpeed(DoubleSupplier speed) {
    tariyaki.set(speed.getAsDouble());
  }
  
//rip

  public void adjustCoral(double meters) {
    double motorRotations = metersToMotorRotations(meters);
    double currentPosition = positionSignal.getValueAsDouble();

    positionVoltage.withPosition(currentPosition + motorRotations);
    tariyaki.setControl(positionVoltage);
  }

  public boolean getFrontBeamBrake()
  {
    return frontBeamBreak.get();
  }
  public boolean getBackBeamBrake()
  {
    return backBeamBreak.get();
  }

  // Helper method to convert meters to ticks
  private double metersToMotorRotations(double meters) {
      // Meters -> Revolutions
      return (meters / wheelCircumference) * gearRatio;
  }
}

