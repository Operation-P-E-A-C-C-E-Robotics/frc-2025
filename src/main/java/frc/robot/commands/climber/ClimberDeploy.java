package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Wrist.WristSetpoints;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ClimberDeploy extends Command {
    private double deployHeight = 10;//TODO
    private Climber climber; // Create an instance variable for Climber

    public ClimberDeploy() {
        climber = new Climber(); // Initialize the Climber instance
        Trigger ClimberDeploy = new JoystickButton(RobotContainer.commandController, frc.robot.Constants.Climber.climberDeployButton);
        //TODO parker you better fix up the constants import (just declare a static constants.climber.* when this is all done)
        if(ClimberDeploy.getAsBoolean()) {
            deploy();
        }
    }

    public void deploy() {
        climber.setPosition(10); // Call setPosition on the Climber instance
    }
}
