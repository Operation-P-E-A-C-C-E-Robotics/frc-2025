// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Reporter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import static frc.robot.Constants.Wrist.*;

public class Wrist extends SubsystemBase {

  /** Creates a new SushiTrain. */
  //operates with "A motor (I get to choose!)" using the reference of a cancoder.
  //not an actual conveyor. Its just the rotating "gear" part of the sushi base so that it can get at the right angles to score.s
  
  private final MotionMagicExpoVoltage motionMagicControl = new MotionMagicExpoVoltage(0);
  private final StatusSignal<Angle> positionSignal;
  private final TalonFX motor = new TalonFX(mainMotorID);
  private final DutyCycleOut dutyCycle = new DutyCycleOut(0);

  public Wrist() {
    Reporter.report(
      motor.getConfigurator().apply(motorConfig),
      "couldn't config elevator master motor"
    );

    positionSignal = motor.getPosition();
    BaseStatusSignal.setUpdateFrequencyForAll(100, 
    positionSignal, 
    motor.getDutyCycle(), 
    motor.getVelocity(), 
    motor.getAcceleration(), 
    motor.getClosedLoopError(),
    motor.getClosedLoopReference()
    );
  }

  /**
   * Sets the speed of the wrist based on arbitrary units
   * @param speed The arbitrary unity in question
   */
  public void setSpeed(double speed)
  {
    motor.setControl(dutyCycle.withOutput(speed));
  }

  /**
   * Sets the climber position based on arbitrary (Rotation2d) units 
   *                                                 -
   * spool rotations are tough to convert
   */
  public void setPosition(Rotation2d angle)
  {
    motionMagicControl.withPosition(angle.getDegrees());
    motor.setControl(motionMagicControl);
  }
  
  /**
   * Returns the wrist motor's position as a Rotation2d
   * @return
   */
  public Rotation2d getClimberPosition()
  {
    return Rotation2d.fromDegrees(motor.getRotorPosition().getValueAsDouble()); //TODO ask sean if when you get the rotor pos as a double, it is converted to degrees
  }
}
