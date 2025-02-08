package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Sushi;
import frc.robot.subsystems.Chute;
import frc.robot.subsystems.Climber;

public class ClimberDeploy extends Command {
    private double deployHeight = 10;//TODO
    private Elevator elevator = new Elevator();
    private Wrist wrist = new Wrist();
    private Chute chute = new Chute();
    private Sushi sushi = new Sushi();
    private Climber climber = new Climber(); // Create an instance variable for Climber

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

    public Command deploy(boolean rightSide) {
     // Call setPosition on the Climber instance
      //elevator not allowed to go up  <- TODO make this, by changing the actual inputs required to 
      //make it go up in some way, once the ele inputs are done
      climber.setPosition(deployHeight);
      return null;
    }

    //Checks to see that the climber has good enough room to deploy
    public boolean deployReady()
    {
        if(10 >= wrist.getWristPosition().getDegrees()) //TODO figure out what angle is "too far back"- 10 is the placeholder atm
        {
            return false;
        }
        if(!sushi.getRearBeamBrake())
        {
            return false;
        }
        if(elevator.getHeight() <= 0.1)
        {
            return false;
        }
        return true;
    }

    //Next meeting standardize this to JUST "deploy"
    // public static Command deployLeftSide(boolean rightSide)
    // {
    //     return
    // }
}
