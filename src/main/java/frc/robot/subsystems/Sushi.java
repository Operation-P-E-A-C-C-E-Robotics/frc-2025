// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.util.Reporter;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import static frc.robot.Constants.Sushi.*;

import java.util.function.BooleanSupplier;

public class Sushi extends SubsystemBase {

  public final TalonFX tariyaki = new TalonFX(sushiMainID);
  private final StatusSignal<Angle> positionSignal;
  private final PositionVoltage positionVoltage = new PositionVoltage(0);

  private DigitalInput frontBeamBreak = new DigitalInput(frontBeamBreakID); // TODO: ask if shawne meant two *Sets* of beam blocks, << yes. "one beam brake sensor" = 1 transmitter + 1 reciever = one digital input
  private DigitalInput rearBeamBreak = new DigitalInput(backBeamBreakID); //or two *Induvidual* beam block sensors


  /**
   * The Sushi is the end effector of the robot responsible for handling coral.
   * It consists of a single motor driving two sets of rollers.
   * It also has two beam brake sensors to allow automatic indexing of coral
   * One is positioned at the front to assist with gamepiece detection with placing,
   * the other is in the rear to ensure the coral clears the elevator when indexed.
   */
  public Sushi() {
    Reporter.report(
      tariyaki.getConfigurator().apply(motorConfig),
      "Failed to configure tariyaki"
    );

    positionSignal = tariyaki.getPosition();

    //optimize can utilization
    ParentDevice.optimizeBusUtilizationForAll(tariyaki);
    BaseStatusSignal.setUpdateFrequencyForAll(100,
      positionSignal,
      tariyaki.getDutyCycle(),
      tariyaki.getVelocity(),
      tariyaki.getAcceleration(),
      tariyaki.getClosedLoopError(),
      tariyaki.getClosedLoopReference()
    );
  }

  /**
   * set the speed of the rollers
   * @param speed Speed from -1 to 1. Positive speeds move the coral torwards the front of the robot
   */
  public void setSpeed(double speed) {
    tariyaki.set(speed);
  }

  /**
   * Use position control to adjust the coral forward or back by a set distance.
   * @param meters How many meters to move the coral. Positive distances move the coral toward the front of the robot.
   */
  public void adjustCoral(double meters) {
    double motorRotations = metersToMotorRotations(meters);
    double currentPosition = positionSignal.getValueAsDouble();

    positionVoltage.withPosition(currentPosition + motorRotations);
    tariyaki.setControl(positionVoltage);
  }

  /**
   * Get whether a gamepiece is blocking the front beam break sensor.
   * The beam break sensor is positioned to detect a coral until it has *almost* fully cleared the sushi.
   * It will be useful for detecting when placement of a coral is complete to automatically retract the elevator.
   */
  public boolean getFrontBeamBrake() {
    return !frontBeamBreak.get();
  }

  /**
   * Get whether a gamepiece is blocking the rear beam break sensor.
   * The sensor is positioned to detect a coral as soon as it enters the sushi,
   * but should NOT be able to see the coral once it is fully in the sushi.
   * It allows us to guarantee that the coral will not collide with the elevator cross bars.
   */
  public boolean getRearBeamBrake() {
    return !rearBeamBreak.get();
  }

  @Override
  public void periodic(){
    SmartDashboard.putBoolean("Front Beam Brake", getFrontBeamBrake());
    SmartDashboard.putBoolean("Rear Beam Brake", getRearBeamBrake());
  }

  public Command place(BooleanSupplier isSlow) {
    return this.run(() -> setSpeed(isSlow.getAsBoolean() ? 0.7:1)).until(() -> !getFrontBeamBrake());
  }

  public Command shuffleBack() {
    return this.run(() -> setSpeed(-0.1)).withTimeout(0.5).andThen(new RunCommand(() -> {}, this));
  }


  /**
   * intake a gamepiece from the chute. only runs if there isn't already a gamepiece in the sushi.
   * @return
   */
  public Command intake() {
    return this.startEnd(
      () -> setSpeed(0.5),  // Start intake at 50% speed
      () -> setSpeed(0)   // Stop when command ends             //Ask shawne if I can just call a command like a normal function
    ).until(this::getRearBeamBrake).unless(this::getFrontBeamBrake); // Stop when coral is detected
  }

  /**
   * Default command. Indexes any gamepiece in the end effector.
   * @return
   */
  public Command index() {
    return this.run(
      () -> setSpeed(getRearBeamBrake() ? 0.1 : 0)
    );
  }

  /**
   * shuffle the gamepiece back for L4 scoring
   * Will only run if the elevator is up as reported by the elevatorDown supplier
   */
  public Command shuffleBack(BooleanSupplier elevatorDown) {
    return this.run(
      () -> setSpeed(getFrontBeamBrake() ? -0.1 : 0)
    ).until(elevatorDown);
  }

  public Command panic() {
    return this.run(
      () -> setSpeed(-0.3)
    );
  }

  // Helper method to convert meters to ticks
  private double metersToMotorRotations(double meters) {
      // Meters -> Revolutions
      return (meters / wheelCircumference) * gearRatio;
  }
}

