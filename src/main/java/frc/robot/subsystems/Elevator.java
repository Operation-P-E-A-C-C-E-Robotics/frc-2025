// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Reporter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import static frc.robot.Constants.Elevator.*;

/*The Elevatoré
 * Two falcon 500s 
 * no available srx on the robot atm, but the can id placeholder is 20 (which we swap out with the sushi's srx can id)
 */
public class Elevator extends SubsystemBase {
  /* HARDS */
  private TalonFX elevatorMaster = new TalonFX(elevatorMasterID); //falcon 500
  private TalonFX elevatorFollower = new TalonFX(elevatorFollowerID);  
  private DigitalInput upperLimitSwitch = new DigitalInput(upperLimitSwitchID); 
  private DigitalInput lowerLimitSwitch = new DigitalInput(lowerLimitSwitchID);

  /* CONTROL MODES */
  private final DutyCycleOut dutyCycle = new DutyCycleOut(0);
  private final MotionMagicExpoVoltage motionMagicControl = new MotionMagicExpoVoltage(0);

  /* STATUS SIGNALS */
  private final StatusSignal<Angle> positionSignal;
  
  /**
   * Is what controls the elevator on the robot, in order to position the sushi at the right height to score
   */
  public Elevator() {
    Reporter.report(
      elevatorMaster.getConfigurator().apply(motorConfig),
      "couldn't config elevator master motor"
    );

    Reporter.report(
      elevatorFollower.getConfigurator().apply(motorConfig),
      "couldn't config elevator follower motor"
    );

    Reporter.report(
      elevatorFollower.setControl(new Follower(elevatorMasterID, false)),
      "failed to configure elevator follow motor to follow master"
    );

    positionSignal = elevatorMaster.getPosition();

    //yer limited in how many CAN signals your motors can send the rio, so to not max it out with unneccessary signals,
    // the first line sets all them to not run, and the second 8 or so set the neccessary ones to run
    // ParentDevice.optimizeBusUtilizationForAll(elevatorMaster, elevatorFollower);
    BaseStatusSignal.setUpdateFrequencyForAll(100, 
      positionSignal,
      elevatorMaster.getDutyCycle(), 
      elevatorMaster.getVelocity(), 
      elevatorMaster.getAcceleration(), 
      elevatorMaster.getClosedLoopError(),
      elevatorMaster.getClosedLoopReference()
    );
  }
  /**
   * Sets the height of the end effector, in meters
   * @param meters How high the end effector is relative to when it was turned on
   */
  public void setHeight(double meters) {
    motionMagicControl.withPosition(heightToSpoolRotations(meters));
    elevatorMaster.setControl(motionMagicControl);
  }
 /**
  * A manual control for the elevator
  * @param value The generic speed at which the elevator moves up and down
  */
  public void setSpeed(double value) {
    elevatorMaster.setControl(dutyCycle.withOutput(value));
  }
 /**
  * Returns the elevators height
  * @return
  */
  public double getHeight() {
    return spoolRotationsToHeight(positionSignal.getValueAsDouble());
  }  
  /**
   * Gets the value the upper limit switch is currently returning
   * @return
   */
  public boolean getUpperLimitSwitch(){
    return upperLimitSwitch.get();
  }
  /**
   * Gets the value the lower limit switch is currently returning
   * @return
   */
  public boolean getLowerLimitSwitch(){
    return upperLimitSwitch.get();
  }
  /**
   * pretty self explanatory innit?
   * @param spoolRotations
   * @return
   */
  private double spoolRotationsToHeight(double spoolRotations) {
    return spoolRotations * spoolCircumference;
  }
  /**
   * Takes in a height value, and returns the amount of the elevator's SpoolRotations
   * @param height
   * @return
   */
  private double heightToSpoolRotations(double height) {
    return height / spoolCircumference;
  }

}
