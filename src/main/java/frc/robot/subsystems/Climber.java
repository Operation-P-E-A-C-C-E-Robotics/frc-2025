// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Climber.*;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Reporter;

public class Climber extends SubsystemBase {
  private final TalonFX leadMotor = new TalonFX(leadClimberMotorID);
  private final TalonFX followerMotor = new TalonFX(followerClimberMotorID);
  /** Creates a new Climber. */
  //literally 2 falcons (with a setpoint and set angle distances from the setpoint, which the motor "counts by")
  public Climber() {
    Reporter.report(
      leadMotor.getConfigurator().apply(motorConfig),
      "couldn't config climber master motor"
    );

    Reporter.report(
      followerMotor.getConfigurator().apply(motorConfig),
      "couldn't config climber follower motor"
    );

    Reporter.report(
      followerMotor.setControl(new Follower(leadClimberMotorID, false)),
      "failed to configure climber follow motor to follow master"
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
