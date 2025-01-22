// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SushiConveyor extends SubsystemBase {

  /** Creates a new SushiTrain. */
  //operates with "A motor (I get to choose!)" using the reference of a cancoder.
  //not an actual conveyor. Its just the rotating "gear" part of the sushi base so that it can get at the right angles to score.s
  public SushiConveyor() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
