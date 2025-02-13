// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Reporter;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import static frc.robot.Constants.Elevator.*;

public class Elevator extends SubsystemBase {

  private final TalonFX master = new TalonFX(elevatorMasterID);
  private final TalonFX follower = new TalonFX(elevatorFollowerID);

  private final DutyCycleOut dutyCycle = new DutyCycleOut(0);
  private final MotionMagicExpoVoltage motionMagic = new MotionMagicExpoVoltage(0);

  private final StatusSignal<Angle> position = master.getPosition();
  private final StatusSignal<ForwardLimitValue> upperLimit = master.getForwardLimit();
  private final StatusSignal<ReverseLimitValue> lowerLimit = master.getReverseLimit();

  public Elevator() {
    Reporter.report(
      master.getConfigurator().apply(motorConfig),
      "couldn't config elevator master motor"
    );

    Reporter.report(
      follower.getConfigurator().apply(motorConfig),
      "couldn't config elevator follower motor"
    );

    Reporter.report(
      follower.setControl(new Follower(elevatorMasterID, false)),
      "failed to configure elevator follow motor to follow master"
    );

    BaseStatusSignal.setUpdateFrequencyForAll(100,
      position,
      master.getDutyCycle(),
      master.getVelocity(),
      master.getAcceleration(),
      master.getClosedLoopError(),
      master.getClosedLoopReference()
    );
  }

  /**
   * Sets the height of the elevator to the given height by using the position control mode of the Talon.
   * @param height the desired height of the elevator in inches
   */
  public void setHeight(double height) {
    motionMagic.withPosition(spoolRotations(height));
    master.setControl(motionMagic);
  }

  /**
   * Sets the speed of the elevator motor.
   * @param speed The desired motor speed between -1 and 1.
   */
  public void setSpeed(double speed) {
    master.setControl(dutyCycle.withOutput(speed));
  }

  public double getHeight() {
    return heightFromSpoolRotations(position.getValueAsDouble());
  }

  public boolean getUpperLimitSwitch() {
    return upperLimit.getValue() == ForwardLimitValue.ClosedToGround;
  }

  public boolean getLowerLimitSwitch() {
    return lowerLimit.getValue() == ReverseLimitValue.ClosedToGround;
  }

  @Override
  public void periodic() {
    // periodically refresh all status signals from the talon, including
    // position, upper limit switch, and lower limit switch
    StatusSignal.refreshAll(position, upperLimit, lowerLimit);
  }

  /**
   * Converts a given height to the equivalent number of spool rotations.
   *
   * @param height the height to convert to spool rotations
   * @return the number of spool rotations corresponding to the given height
   */
  private double spoolRotations(double height) {
    return height / spoolCircumference;
  }

  /**
   * @param rotations the spool rotations to convert to a height
   * @return the height corresponding to the given spool rotations
   */
  private double heightFromSpoolRotations(double rotations) {
    return rotations * spoolCircumference;
  }
}

