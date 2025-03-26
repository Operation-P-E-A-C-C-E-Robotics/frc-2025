package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.drivetrain.AutoAlign;
import frc.robot.subsystems.Chute;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Elevator.ElevatorSetpoints;
import frc.robot.subsystems.Sushi;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist.WristSetpoints;

public class AutomationCommands {
    private final Elevator elevator;
    private final Wrist wrist;
    private final Sushi sushi;
    private final Chute chute;
    private final AutoAlign autoAlign;

    public AutomationCommands(Swerve swerve, Elevator elevator, Wrist wrist, Sushi sushi, Chute chute, AutoAlign autoAlign) {
        this.elevator = elevator;
        this.wrist = wrist;
        this.sushi = sushi;
        this.chute = chute;
        this.autoAlign = autoAlign;
        // NamedCommands.registerCommand("L1", l1ElevatorWrist().alongWith(elevator.goToSetpoint(ElevatorSetpoints.L1)));
        // NamedCommands.registerCommand("L2", l2ElevatorWrist().alongWith(elevator.goToSetpoint(ElevatorSetpoints.L2)));
        // NamedCommands.registerCommand("L4", l4ElevatorWrist().alongWith(elevator.goToSetpoint(ElevatorSetpoints.L4)));
    }
    public Command l1ElevatorWrist() {
        return elevator.goToSetpoint(ElevatorSetpoints.L1).alongWith(wrist.goToSetpoint(WristSetpoints.L1));
    }
    public Command l2ElevatorWrist() {
        return elevator.goToSetpoint(ElevatorSetpoints.L2).alongWith(wrist.goToSetpoint(WristSetpoints.L2L3)).until(() -> !sushi.getFrontBeamBrake());
    }
    public Command l2_5ElevatorWrist() {
        return elevator.goToSetpoint(ElevatorSetpoints.L2_5, true).alongWith(wrist.goToSetpoint(WristSetpoints.L2L3)).alongWith(sushi.manualInput(() -> 0.8));
    }
    public Command l3ElevatorWrist() {
        return elevator.goToSetpoint(ElevatorSetpoints.L3).alongWith(wrist.goToSetpoint(WristSetpoints.L2L3)).until(() -> !sushi.getFrontBeamBrake());
    }
    public Command l3_5ElevatorWrist() {
        return elevator.goToSetpoint(ElevatorSetpoints.L3_5, true).alongWith(wrist.goToSetpoint(WristSetpoints.L2L3)).alongWith(sushi.manualInput(() -> 0.8));
    }
    public Command l4ElevatorWrist() {
        return elevator.goToSetpoint(ElevatorSetpoints.L4)
            .alongWith(new RunCommand(() -> {}, wrist)
                .until(() -> elevator.getHeight() > 7.9 - Constants.Elevator.setpointTolerance)
                .withTimeout(5)  //A "Nothing" function is looped so the elevator can get to the right point before adjusting the wrist angle
            .andThen(
                wrist.goToSetpoint(WristSetpoints.L4)
                .alongWith(sushi.shuffleBack(() -> elevator.getHeight() < 7.9 - Constants.Elevator.setpointTolerance))));//.until(() -> !(sushi.getFrontBeamBrake() || sushi.getRearBeamBrake()));
    }
    public Command l2AutoAlign() {
        return autoAlign.alongWith(
            new WaitUntilCommand(autoAlign::aligning)
            .andThen(l2ElevatorWrist())
        );
    }
    public Command l3AutoAlign() {
        return autoAlign.alongWith(
            new WaitUntilCommand(autoAlign::aligning)
            .andThen(l3ElevatorWrist())
        );
    }

    public Command l4AutoAlign() {
        return autoAlign.alongWith(
            new WaitUntilCommand(autoAlign::aligning)
            .andThen(l4ElevatorWrist())
        );
    }

    // public Command placeAndRetract() {
    //     return sushi.place(() -> false).andThen(new WaitCommand(0.4)).andThen(elevator.goToSetpoint(ElevatorSetpoints.REST).alongWith(wrist.goToSetpoint(WristSetpoints.REST)));
    // }

    // public Co 

    public Command intakeUntilCoralObtained() {
        return new ParallelRaceGroup(chute.intake(), sushi.intake());
    }
}
