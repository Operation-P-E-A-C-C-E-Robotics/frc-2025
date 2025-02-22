// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import static frc.robot.Constants.Chute.*;

public class Chute extends SubsystemBase {

    public static TalonSRX rightChuteMotor = new TalonSRX(rightMotorID); // Should be 28
    public static TalonSRX leftChuteMotor = new TalonSRX(leftMotorID);   // Should be 16
    public static TalonSRX chuteDropMotor = new TalonSRX(6); // TODO

    private boolean hasDropped = false;

    public Chute() {
        rightChuteMotor.setInverted(false);
        rightChuteMotor.configPeakCurrentLimit(40);

        leftChuteMotor.setInverted(false);
        leftChuteMotor.configPeakCurrentLimit(40);

        chuteDropMotor.setInverted(false);
        chuteDropMotor.configPeakCurrentLimit(40);
    }

    /**
     * Sets the speed of the left and right chute motors.
     * 
     * @param leftSpeed  The speed of the left motor.
     * @param rightSpeed The speed of the right motor.
     */
    public void setSpeed(double leftSpeed, double rightSpeed) {
        leftChuteMotor.set(ControlMode.PercentOutput, leftSpeed);
        rightChuteMotor.set(ControlMode.PercentOutput, rightSpeed);
    }

    public double getLeftCurrent() {
        return leftChuteMotor.getSupplyCurrent();
    }

    public double getRightCurrent() {
        return rightChuteMotor.getSupplyCurrent();
    }

    /**
     * Activates the "Drop motor" to run backwards. This pulls the pin holding the chute up, allowing it to drop.
     */
    public boolean hasDropped() {
        return hasDropped;
    }

    public Command dropCommand() {
        return this.run(() -> {
            chuteDropMotor.set(ControlMode.PercentOutput, 0.4); // TODO: Figure out if this is a good speed to pull the pins
            hasDropped = true;
        });
    }

    public Command intake() {
        return this.runOnce(() -> setSpeed(intakeSpeed, intakeSpeed));
    }

    public Command unjam() {
        return this.runOnce(() -> setSpeed(unjamSpeed, unjamSpeed));
    }
}

