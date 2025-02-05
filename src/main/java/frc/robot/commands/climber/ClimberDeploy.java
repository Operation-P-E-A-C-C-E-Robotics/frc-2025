package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Climber;

public class ClimberDeploy extends Command {
    private double deployHeight = 10;//TODO
    private Climber climber; // Create an instance variable for Climber

    /*
     *  If the wrist is too far back, elevator not allowed to go up.
        If the wrist is too far out, elevator cant go down (biggest factor when placing)
        If coral is not fully indexed, elevator not allowed to go up. / -  
        Elevator needs to be all the way down when climber is deployed
        Elevator cannot go up when climber is deployed

     */

    public ClimberDeploy() {
        climber = new Climber(); // Initialize the Climber instance
        // Trigger ClimberDeploy = new JoystickButton(RobotContainer.commandController, frc.robot.Constants.Climber.climberDeployButton);
        // TODO parker you better fix up the constants import (just declare a static constants.climber.* when this is all done)
        // if(ClimberDeploy.getAsBoolean()) {
        //     deploy();
        // }
    }

    public void deploy() {
      climber.setPosition(10); // Call setPosition on the Climber instance
    }

    public boolean climberDeployed = false;
    //Next meeting standardize this to JUST "deploy"
    // public static Command deployLeftSide(boolean rightSide)
    // {
    //     return
    // }
}
