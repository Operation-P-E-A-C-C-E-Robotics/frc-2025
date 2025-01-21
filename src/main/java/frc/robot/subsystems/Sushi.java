// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

public class Sushi extends Command {
  private final TalonFX spinner = new TalonFX(99);

  /** Creates a new Sushi. */
  //Set up literally one falcon 500, that (theoretically) triggers when you press a button/trigger. You can do this!!!
  // Backwards AND forwards
  public Sushi() {}

  public void SoySauce(BooleanSupplier bool) {
    if (bool.getAsBoolean()) {
      spinner.set(0);
    }
  }
}
