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
import frc.robot.RobotContainer;

public class Climber extends SubsystemBase {
  private final MotionMagicExpoVoltage motionMagicControl = new MotionMagicExpoVoltage(0);
  private final StatusSignal<Angle> positionSignal;
  private final DutyCycleOut dutyCycle = new DutyCycleOut(0);
  private final RobotContainer robotContainer = new RobotContainer();
  private final TalonFX climbMaster = new TalonFX(leadClimberMotorID);

  private final Elevator elevator = new Elevator();
  private final Wrist wrist = new Wrist();
  private final Sushi sushi = new Sushi();

  private final double maxSpeed = 0.8;
  private final double deployHeight = 10; // TODO: Determine the correct deploy height
  public boolean deployed = false;

  public Climber() {
    positionSignal = climbMaster.getPosition();
    
    // Set update frequency for relevant signals
    BaseStatusSignal.setUpdateFrequencyForAll(100,
      positionSignal,
      climbMaster.getDutyCycle(),
      climbMaster.getVelocity(),
      climbMaster.getAcceleration(),
      climbMaster.getClosedLoopError(),
      climbMaster.getClosedLoopReference()
    );
  }

  /**
   * Checks if the climber is ready to deploy.
   * 
   * @return true if the climber is ready to deploy, false otherwise.
   */
  public boolean deployReady() {
    // Ensure wrist is not too far back
    if (wrist.getWristPosition().getDegrees() <= 10) { // TODO: Adjust angle threshold
      return false;
    }
    
    // Ensure there is no obstruction detected by the rear beam brake
    if (sushi.getRearBeamBrake()) {
      return false;
    }

    // Ensure the elevator is at a minimum height
    return elevator.getHeight() > 0.1; //TODO: if theres any elevator input issues it might be from this
  }

  /**
   * Sets the climber motor speed.
   * 
   * @param speed The desired motor speed.
   */
  public void setSpeed(double speed) {
    if (deployed) {
      climbMaster.setControl(dutyCycle.withOutput(speed));
    }
  }

  /**
   * Moves the climber to a specific position.
   * 
   * @param position The desired position.
   */
  public void setPosition(double position) {
    climbMaster.setControl(motionMagicControl.withPosition(position));
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
    if (getPosition() < (deployHeight - 0.01)) {
      setPosition(0); // TODO: Determine the correct climb position
    }
    return null;
  }

  /**
   * Allows manual control of the climber using a joystick axis.
   * 
   * @param speed A supplier providing the desired speed.
   * @return A command that allows manual control of the climber.
   */
  public Command manualInput(DoubleSupplier speed) {
    return this.run(() -> setSpeed(speed.getAsDouble())); // TODO: Verify correct axis mapping
  }

  /**
   * Deploys the climber if all conditions are met.
   * 
   * @return A command to deploy the climber.
   */
  public Command deploy() 
  {
    return this.run(() -> {
      if(robotContainer.deployReady()) {
        setPosition(deployHeight);
        deployed = true;
      }
    });
  }

  /**
   * Commands the climber to climb up the tower at a specified speed.
   * 
   * @param speed The desired climbing speed.
   * @return A command that controls the climber movement.
   */
  public Command climb(double speed) {
    setSpeed(speed * maxSpeed);
    return null;
  }
}
