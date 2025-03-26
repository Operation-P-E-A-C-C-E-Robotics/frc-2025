// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Climber.autoClimbPosition;
import static frc.robot.Constants.Climber.autoClimbSpeed;
import static frc.robot.Constants.Climber.climberDeployMotorID;
import static frc.robot.Constants.Climber.climberWinchMotorID;
import static frc.robot.Constants.Climber.deployConfig;
import static frc.robot.Constants.Climber.winchConfig;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final TalonFX climbWinch = new TalonFX(climberWinchMotorID);
  private final TalonFX climbDeploy = new TalonFX(climberDeployMotorID);

  private final StatusSignal<Angle> positionSignal;
  private final BooleanSupplier deployReady, chuteDropped;
  private boolean deployed = false;

  public Climber(BooleanSupplier deployReady, BooleanSupplier chuteDropped) {
    this.deployReady = deployReady;
    this.chuteDropped = chuteDropped;
  
    positionSignal = climbDeploy.getPosition();
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
    //   climbWinch.getClosedLoopReference(),
    //   climbDeploy.getDutyCycle(),
    //   climbDeploy.getVelocity(),
    //   climbDeploy.getAcceleration(),
    //   climbDeploy.getClosedLoopError(),
    //   climbDeploy.getClosedLoopReference(),
    // );
  }


  /**
   * Sets the deploy motor speed.
   * 
   * @param speed The desired motor speed - negative values to deploy
   */
  public void setDeploySpeed(double speed) {
    climbDeploy.set(speed);
  }

  /**
   * Sets the winch motor speed.
   * 
   * @param speed The desired motor speed - positive values to climb
   */
  public void setWinchSpeed(double speed) {
    climbWinch.set(speed);
  }

  /**
   * Moves the climber to a specific position.
   * 
   * @param position The desired position.
   */
  public void winchToPosition(double position) {
    if(getPosition() < position) {
      setWinchSpeed(autoClimbSpeed);
      setDeploySpeed(1);
    } else {
      setWinchSpeed(0);
      setDeploySpeed(0);
    }
  }

  /**
   * Gets the current climber position.
   * 
   * @return The current position of the climber.
   */
  public double getPosition() {
    return positionSignal.getValueAsDouble();
  }

  public boolean hasDeployed() {
    return deployed;
  }

  /**
   * Stops the climber by setting its speed to zero.
   * 
   * @return A command that stops the climber.
   */
  public Command rest() {
    return this.run(() -> {
      setDeploySpeed(0);
      setWinchSpeed(0);
    });
  }

  /**
   * Moves the climber to the climb position.
   * 
   * @return A command to move the climber to the climb position.
   */
  public Command getToClimbPos() {
    return this.run(() -> winchToPosition(autoClimbPosition));
  }


  /**
   * Deploys the climber if all conditions are met.
   * 
   * @return A command to deploy the climber.
   */
  public Command deploy() {
    return this.run(() -> {
      // if(deployReady.getAsBoolean()) {
        setDeploySpeed(-1);
        deployed = true;
      // }
    });
  }

  /**
   * Allows manual control of the climber using a joystick axis.
   * 
   * @param speed A supplier providing the desired speed.
   * @return A command that allows manual control of the climber.
   */
  public Command manualInput(DoubleSupplier speedSupplier) {
    return this.run(() ->  {
      // if(!deployed) return;
      var speed = speedSupplier.getAsDouble();
      // if(speed > 0 && !chuteDropped.getAsBoolean() && getPosition() > 0) speed = 0;
      // setDeploySpeed(speed);
      setWinchSpeed(speed);
      setDeploySpeed(0);
    });//.unless(() -> !deployed);
  }
}
