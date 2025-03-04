// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Elevator extends SubsystemBase {

  private final TalonFX master = new TalonFX(elevatorMasterID);
  private final TalonFX follower = new TalonFX(elevatorFollowerID);

  private final DutyCycleOut dutyCycle = new DutyCycleOut(0);
  private final MotionMagicExpoVoltage motionMagic = new MotionMagicExpoVoltage(0);

  private double lastHeight = 0;

  private final StatusSignal<Angle> position;
  private final StatusSignal<ForwardLimitValue> upperLimit = master.getForwardLimit();
  private final StatusSignal<ReverseLimitValue> lowerLimit = master.getReverseLimit();
  private final BooleanSupplier coralIndexed, wristExtended;
      
  public Elevator(BooleanSupplier coralIndexed, BooleanSupplier wristExtended) {
    this.coralIndexed = coralIndexed;
    this.wristExtended = wristExtended;

    position = master.getPosition();

    Reporter.report(
      master.getConfigurator().apply(motorConfig),
      "couldn't config elevator master motor"
    );

    Reporter.report(
      follower.getConfigurator().apply(motorConfig),
      "couldn't config elevator follower motor"
    );

    Reporter.report(
      follower.setControl(new Follower(elevatorMasterID, true)),
      "failed to configure elevator follow motor to follow master"
    );

    BaseStatusSignal.setUpdateFrequencyForAll(100,
      position,
      master.getDutyCycle(),
      master.getVelocity(),
      master.getAcceleration(),
      master.getClosedLoopError(),
      master.getClosedLoopReference(),
      upperLimit,
      lowerLimit
    );
  }

  /**
   * Sets the height of the elevator to the given height by using the position control mode of the Talon.
   * @param height the desired height of the elevator in inches
   */
  public void setHeight(double height) {
    if(!coralIndexed.getAsBoolean() && height > maxExtensionWithoutIndexing) height = maxExtensionWithoutIndexing;
    if(wristExtended.getAsBoolean() && height < lastHeight) return;

    // motionMagic.withSlot(height >= lastHeight ? 0 : 1);
    motionMagic.withPosition(motorRotationsFromHeight(height));
    master.setControl(motionMagic);
    lastHeight = height;
  }

  /**
   * Sets the speed of the elevator motor.
   * @param speed The desired motor speed between -1 and 1.
   */
  public void setSpeed(double speed) {
    if(!coralIndexed.getAsBoolean() && getHeight() > maxExtensionWithoutIndexing && speed > 0) speed = 0;
    if(wristExtended.getAsBoolean() && speed < 0) speed = 0;
    master.setControl(dutyCycle.withOutput(speed).withOverrideBrakeDurNeutral(true));
  }

  public double getHeight() {
    return heightFromMotorRotations(position.getValue().baseUnitMagnitude());
  }

  public boolean getUpperLimitSwitch() {
    return upperLimit.getValue() == ForwardLimitValue.ClosedToGround;
  }

  public boolean getLowerLimitSwitch() {
    return lowerLimit.getValue() == ReverseLimitValue.ClosedToGround;
  }

  public Command goToSetpoint(ElevatorSetpoints setpoint) {
      return this.run(() -> setHeight(setpoint.getHeight()));
  }

  public Command manualInput(DoubleSupplier speed) {
      return this.run(() -> setSpeed(speed.getAsDouble()));
  }

  @Override
  public void periodic() {
    // periodically refresh all status signals from the talon, including
    // position, upper limit switch, and lower limit switch
    StatusSignal.refreshAll(position, upperLimit, lowerLimit);
    SmartDashboard.putNumber("Elevator Height Meters", getHeight());
    SmartDashboard.putBoolean("Elevator Upper Limit", getUpperLimitSwitch());
    SmartDashboard.putBoolean("Elevator Lower Limit", getLowerLimitSwitch());    
  }

  /**
   * Converts a given height to the equivalent number of spool rotations.
   *
   * @param height the height to convert to spool rotations
   * @return the number of spool rotations corresponding to the given height
   */
  private double motorRotationsFromHeight(double height) {
    return (height / spoolCircumference) / spoolRotationsPerMotorRotations;
  }

  /**
   * @param rotations the spool rotations to convert to a height
   * @return the height corresponding to the given spool rotations
   */
  private double heightFromMotorRotations(double rotations) {
    return (rotations * spoolRotationsPerMotorRotations) * spoolCircumference;
  }

  public enum ElevatorSetpoints {
        REST(0.0),
        L1(0.13),
        L2(0.35),
        L3(0.75),
        L4(1.3);

        private double height;

        ElevatorSetpoints(double height) {
            this.height = height;
        }

        public double getHeight() {
            return height;
        }
    }
}

