package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Chute;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Elevator.ElevatorSetpoints;
import frc.robot.subsystems.Sushi;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist.WristSetpoints;

public class AutomationCommands {
    private final Swerve swerve;
    private final Elevator elevator;
    private final Wrist wrist;
    private final Sushi sushi;
    private final Climber climber;
    private final Chute chute;

    public AutomationCommands(Swerve swerve, Elevator elevator, Wrist wrist, Sushi sushi, Climber climber, Chute chute) {
        this.swerve = swerve;
        this.elevator = elevator;
        this.wrist = wrist;
        this.sushi = sushi;
        this.climber = climber;
        this.chute = chute;
    }

    public Command l1ElevatorWrist() {
        return elevator.goToSetpoint(ElevatorSetpoints.L1).alongWith(wrist.goToSetpoint(WristSetpoints.L1));
    }

    public Command l2ElevatorWrist() {
        return elevator.goToSetpoint(ElevatorSetpoints.L2).alongWith(wrist.goToSetpoint(WristSetpoints.L2L3)).until(() -> !sushi.getFrontBeamBrake());
    }

    public Command l3ElevatorWrist() {
        return elevator.goToSetpoint(ElevatorSetpoints.L3).alongWith(wrist.goToSetpoint(WristSetpoints.L2L3)).until(() -> !sushi.getFrontBeamBrake());
    }

    public Command l4ElevatorWrist() {
        return elevator.goToSetpoint(ElevatorSetpoints.L4)
            .alongWith(new RunCommand(() -> {}, wrist)
                .until(() -> elevator.getHeight() > ElevatorSetpoints.L4.getHeight() - 0.05)
                .withTimeout(3)  //A "Nothing" function is looped for 3 seconds so the elevator can get to the right point, and then it sets the wrist angle.
            .andThen(wrist.goToSetpoint(WristSetpoints.L4))).until(() -> !sushi.getFrontBeamBrake());
    }

    public Command placeAndRetract() {
        return sushi.place().andThen(new WaitCommand(0.4)).andThen(elevator.goToSetpoint(ElevatorSetpoints.REST).alongWith(wrist.goToSetpoint(WristSetpoints.REST)));
    }

    public Command deployClimber() {
        return chute.dropCommand().alongWith(climber.deploy()).withTimeout(2);
    }

    public Command intakeUntilCoralObtained() {
        return new ParallelRaceGroup(chute.intake(), sushi.intake());
    }
}
