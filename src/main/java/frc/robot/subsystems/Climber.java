// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Climber.climberDeployButtonLeft;
import static frc.robot.Constants.Climber.climberDeployMotorID;
import static frc.robot.Constants.Climber.climberWinchMotorID;
import static frc.robot.Constants.Climber.deployConfig;
import static frc.robot.Constants.Climber.winchConfig;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Climber extends SubsystemBase {
  private final TalonFX climbWinch = new TalonFX(climberWinchMotorID);
  private final TalonFX climbDeploy = new TalonFX(climberDeployMotorID);
  private final MotionMagicExpoVoltage motionMagicControl = new MotionMagicExpoVoltage(0);

  private final StatusSignal<Angle> positionSignal;
  private final DutyCycleOut dutyCycle = new DutyCycleOut(0);

  private final double deploySetpoint = 10; // TODO: Determine the correct deploy height
  private final BooleanSupplier deployReady;

  private boolean deployed = false;

  public Climber(BooleanSupplier deployReady) {
    this.deployReady = deployReady;
  
    positionSignal = climbWinch.getPosition();
    climbWinch.getConfigurator().apply(winchConfig);
    climbDeploy.getConfigurator().apply(deployConfig);
    
    // Set update frequency for relevant signals
    // ParentDevice.optimizeBusUtilizationForAll(climbWinch, climbDeploy);
    // BaseStatusSignal.setUpdateFrequencyForAll(100,
    //   positionSignal,
    //   climbWinch.getDutyCycle(),
    //   climbWinch.getVelocity(),
    //   climbWinch.getAcceleration(),
    //   climbWinch.getClosedLoopError(),
    //   climbWinch.getClosedLoopReference()
    // );
  }


  /**
   * Sets the climber motor speed.
   * 
   * @param speed The desired motor speed.
   */
  public void setSpeed(double speed) {
    // if (deployed) {
    //   if(speed < 0)
    //   {
    //     if(elevator.getHeight() >= 0.1)
    //     {
    //       climbWinch.setControl(dutyCycle.withOutput(speed));    
    //     }
    //   }
    //   else
    //   {
    //     climbWinch.setControl(dutyCycle.withOutput(speed));        
    //   }
    // }
    dutyCycle.withOutput(speed);
    climbWinch.setControl(dutyCycle);
    climbDeploy.setControl(dutyCycle);
  }

  /**
   * Moves the climber to a specific position.
   * 
   * @param position The desired position.
   */
  public void setPosition(double position) {
    climbWinch.setControl(motionMagicControl.withPosition(position));
  }

  /**
   * Gets the current climber position.
   * 
   * @return The current position of the climber.
   */
  public double getPosition() {
    return positionSignal.getValueAsDouble();
  }

  /**
   * Stops the climber by setting its speed to zero.
   * 
   * @return A command that stops the climber.
   */
  public Command rest() {
    return this.run(() -> setSpeed(0));
  }

  /**
   * Moves the climber to the climb position.
   * 
   * @return A command to move the climber to the climb position.
   */
  public Command getToClimbPos() {
    if (getPosition() < (deploySetpoint - 0.01)) {
      setPosition(0); // TODO: Determine the correct climb position
    }
    return null;
  }


  /**
   * Deploys the climber if all conditions are met.
   * 
   * @return A command to deploy the climber.
   */
  public Command deploy() {
    return this.run(() -> {
      if(deployReady.getAsBoolean()) {
        setPosition(deploySetpoint);
        deployed = true;
      }
    });
  }

  /**
   * Allows manual control of the climber using a joystick axis.
   * 
   * @param speed A supplier providing the desired speed.
   * @return A command that allows manual control of the climber.
   */
  public Command manualInput(DoubleSupplier speed) {
    return this.run(() ->  setSpeed(speed.getAsDouble()));
  }
}
