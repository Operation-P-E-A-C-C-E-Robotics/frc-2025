// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Reporter;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

import static frc.robot.Constants.Wrist.*;

public class Wrist extends SubsystemBase {
  
  private final MotionMagicExpoVoltage motionMagicControl = new MotionMagicExpoVoltage(0);
  private final StatusSignal<Angle> positionSignal;
  private final TalonFX motor = new TalonFX(mainMotorID);
  private final DutyCycleOut dutyCycle = new DutyCycleOut(0);
  

  public Wrist() {
    // Trigger WristRestingSetpoint = new JoystickButton(RobotContainer.driverController, restButtonID); WristRestingSetpoint.onTrue(GoToSetpoint(WristSetpoints.REST));
    // Trigger WristL1Setpoint = new JoystickButton(RobotContainer.driverController, L1ButtonID); WristL1Setpoint.onTrue(GoToSetpoint(WristSetpoints.L1));
    // Trigger WristL2L3Setpoint = new JoystickButton(RobotContainer.driverController, L2L3ButtonID); WristL2L3Setpoint.onTrue(GoToSetpoint(WristSetpoints.L2L3));
    // Trigger WristL4Setpoint = new JoystickButton(RobotContainer.driverController, L4ButtonID); WristL4Setpoint.onTrue(GoToSetpoint(WristSetpoints.L4));


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
   * Sets the speed of the wrist based on duty cycle
   */
  public void setSpeed(double speed) {
    motor.setControl(dutyCycle.withOutput(speed));
  }

  /** 
   * 
   */
  public void setPosition(Rotation2d angle) {
    motionMagicControl.withPosition(angle.getDegrees());
    motor.setControl(motionMagicControl);
  }
  
  /**
   * Returns the wrist motor's position as a Rotation2d
   * @return
   */
  public Rotation2d getWristPosition(){  //Sheahne why is it going to a rest setpoint first?
    return Rotation2d.fromRotations(motor.getRotorPosition().getValueAsDouble()); //TODO ask sean if when you get the rotor pos as a double, it is converted to degrees
  }
  
  public Command goToSetpoint(WristSetpoints setpoint){
    return this.runOnce(() -> setPosition(setpoint.getAngle()));
  }

  public enum WristSetpoints {
    REST  (Rotation2d.fromDegrees(0)),
    L1    (Rotation2d.fromDegrees(35)),
    L2L3  (Rotation2d.fromDegrees(75)),
    L4    (Rotation2d.fromDegrees(90));

    private Rotation2d angle;
    private WristSetpoints (Rotation2d angle) {
      this.angle = angle;
    }

    public Rotation2d getAngle() {
      return angle;
    }
  }
}
