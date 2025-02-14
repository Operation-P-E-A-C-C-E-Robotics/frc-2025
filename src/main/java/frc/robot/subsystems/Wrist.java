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

import java.util.function.DoubleSupplier;

public class Wrist extends SubsystemBase {

    private final MotionMagicExpoVoltage motionMagicControl = new MotionMagicExpoVoltage(0);
    private final StatusSignal<Angle> positionSignal;
    private final TalonFX motor = new TalonFX(mainMotorID);
    private final DutyCycleOut dutyCycle = new DutyCycleOut(0);

    public Wrist() {
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
     * Sets the speed of the wrist by adjusting the duty cycle.
     * 
     * @param speed The desired speed for the wrist motor, represented as a duty cycle value.
     */
    public void setSpeed(double speed) {
        motor.setControl(dutyCycle.withOutput(speed));
    }

    /**
     * Sets the position of the wrist using Motion Magic control
     * 
     * Motion Magic is a position control strategy implemented by CTRE that
     * takes into account the motor's acceleration and velocity limits to generate
     * a smooth and efficient motion profile. This method takes a Rotation2d object
     * representing the desired wrist angle and uses it to set the Motion Magic
     * control's position setpoint.
     * 
     * @param angle the desired wrist angle as a Rotation2d
     */
    public void setPosition(Rotation2d angle) {
        motionMagicControl.withPosition(angle.getDegrees());
        motor.setControl(motionMagicControl);
    }

    /**
     * Retrieves the current position of the wrist motor.
     * 
     * This method converts the motor's rotor position into a Rotation2d object,
     * which provides a convenient way to represent and work with angular positions
     * in the codebase.
     * 
     * @return The current wrist position as a Rotation2d.
     */
    public Rotation2d getWristPosition() {
        double rotorPosition = motor.getRotorPosition().getValueAsDouble();
        return Rotation2d.fromRotations(rotorPosition);
    }

    /**
     * Returns a command to move the wrist to a specific setpoint
     * 
     * This method takes a WristSetpoints enum value and returns a command that
     * will move the wrist to the corresponding setpoint. The command is
     * implemented using the runOnce() method, which will execute the given
     * lambda expression once and then finish immediately.
     * 
     * @param setpoint the desired wrist setpoint
     * @return a command to move the wrist to the setpoint
     */
    public Command goToSetpoint(WristSetpoints setpoint) {
        return this.runOnce(() -> setPosition(setpoint.getAngle()));
    }

    public enum WristSetpoints {
        REST(Rotation2d.fromDegrees(0)),
        L1(Rotation2d.fromDegrees(35)),
        L2L3(Rotation2d.fromDegrees(75)),
        L4(Rotation2d.fromDegrees(90));

        private Rotation2d angle;

        WristSetpoints(Rotation2d angle) {
            this.angle = angle;
        }

        public Rotation2d getAngle() {
            return angle;
        }
    }

    public Wrist manualInput(DoubleSupplier speed) {
    setSpeed(speed.getAsDouble());
    return this; // TODO: Verify correct axis mapping
    }
}
