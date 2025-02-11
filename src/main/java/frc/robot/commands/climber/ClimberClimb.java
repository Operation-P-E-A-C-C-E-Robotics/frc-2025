package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberClimb extends Command{
    private ClimberDeploy climberDeploy = new ClimberDeploy();
    private Climber climber = new Climber();
    public ClimberClimb()
    {   

    }
    
    public Command getToClimbPos()
    {
        if(climber.getPosition() < (climberDeploy.deployHeight - 0.01))
        climber.setPosition(0); //TODO figure out where climb pos is
        return null;
    }
}