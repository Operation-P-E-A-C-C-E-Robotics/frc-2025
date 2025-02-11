package frc.robot.commands.climber;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.climber.ClimberDeploy;

public class ManualClimb extends Command{
    private double maxSpeed = 0.8;
    private Climber climber = new Climber();
    private ClimberDeploy climberDeploy = new ClimberDeploy();
    public ManualClimb(){
        
    }
    
    public Command climb(double speed){
        climber.setSpeed(speed * maxSpeed);
        return null;
    }
    
}
