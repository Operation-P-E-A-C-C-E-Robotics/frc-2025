// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutomationCommands;
import frc.robot.commands.drivetrain.PeaccyDrive;
import frc.robot.subsystems.Sushi;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Elevator.ElevatorSetpoints;
import frc.lib.util.ButtonMap;
import frc.lib.util.ButtonMap.Button;
import frc.lib.util.ButtonMap.JoystickTrigger;
import frc.lib.util.ButtonMap.MultiButton;
import frc.lib.util.ButtonMap.OIEntry;
import frc.robot.subsystems.Wrist.WristSetpoints;
import frc.robot.subsystems.Chute;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import static frc.robot.Constants.Wrist.*;


public class RobotContainer {
  /* OI CONSTANTS */

  /* SUBSYSTEMS */
  private final Sushi sushi = new Sushi();
  private final Wrist wrist = new Wrist();
  private final Elevator elevator = new Elevator(() -> !sushi.getRearBeamBrake(), () -> wrist.getWristPosition().getDegrees() > 60);
  private final Chute chute = new Chute();
  private final Climber climber = new Climber(this::deployReady, chute::hasDropped);
  private final Swerve driveTrain = new Swerve ();

  public AutomationCommands automationCommands = new AutomationCommands(driveTrain, elevator, wrist, sushi, climber, chute);

  // private final DriveTrainTuner driveTrainTuneable = new DriveTrainTuner();

  /* OI DEFINITIONS */
  

  private final Joystick driverJoystick = new Joystick(0);
  private final Joystick operatorJoystick = new Joystick(1);

  private final JoystickButton zeroButton = new JoystickButton(driverJoystick, Constants.OI.zeroOdometry); //for debugging

  private final OIEntry[] operatorsMap = new OIEntry[] {
    //Copy-Paste controls in here
    //----------------------------------------------------------------------------------------------------------------------------------------
    
    Button.onHold(sushi.place(() -> elevator.getHeight() < 1.5), 8),
    Button.onHold(sushi.intake(), 7),
    Button.onHold(chute.intake(), 7),
    Button.onRelease(chute.rest(), 7),
    Button.onHold(chute.unjam().alongWith(sushi.panic()), 5),
    Button.onRelease(chute.rest(), 5),
    // Button.onPress(climber.getToClimbPos(), 6),

    Button.onHold(automationCommands.l1ElevatorWrist(), 3),
    Button.onHold(automationCommands.l2ElevatorWrist(), 2),
    Button.onHold(automationCommands.l3ElevatorWrist(), 1),
    Button.onHold(automationCommands.l4ElevatorWrist(), 4),

    // MultiButton.onHold(automationCommands.deployClimber(), 9, 10),

    Button.onPress(elevator.goToSetpoint(ElevatorSetpoints.REST),12),
    Button.onPress(wrist.goToSetpoint(WristSetpoints.REST),12),

    JoystickTrigger.onMove(operatorJoystick, elevator.manualInput(() -> -operatorJoystick.getRawAxis(3)*1), 3, 0.15),
    JoystickTrigger.onMove(operatorJoystick, wrist.manualInput(() -> -operatorJoystick.getRawAxis(1)*1), 1, 0.1),
    JoystickTrigger.onMove(operatorJoystick, climber.manualInput(() -> -operatorJoystick.getRawAxis(0)*0.05), 0, 0.2),
  };

  private final OIEntry[] driverMap = new OIEntry[] {
    JoystickTrigger.onMove(driverJoystick, automationCommands.intakeUntilCoralObtained(), 2, 0.5),
    JoystickTrigger.onZero(driverJoystick, sushi.index().alongWith(chute.rest()), 2, 0.4),
    JoystickTrigger.onMove(driverJoystick, sushi.place(() -> elevator.getHeight() < 1.0), 3, 0.5),

    Button.onHold(automationCommands.l1ElevatorWrist(), 2),
    Button.onHold(automationCommands.l2ElevatorWrist(), 1),
    Button.onHold(automationCommands.l3ElevatorWrist(), 3),
    Button.onHold(automationCommands.l4ElevatorWrist(), 4),
  };

  //-------------------------------------------------------------------------------------------------------------------------------------


  /* COMMANDS */
  private final PeaccyDrive peaccyDrive = new PeaccyDrive(driveTrain);

  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

  public RobotContainer() {
    configureBindings();
    SmartDashboard.putData("AUTO MODE", autoChooser);
  }

  private void configureBindings() {
    peaccyDrive.withTranslation(() -> -driverJoystick.getRawAxis(Constants.OI.translationAxis) * 0.7)
               .withStrafe     (() -> -driverJoystick.getRawAxis(Constants.OI.strafeAxis) * 0.7)
               .withRotation   (() -> -driverJoystick.getRawAxis(Constants.OI.rotationAxis))
               .withHeading    (() -> (double) -driverJoystick.getPOV())
               .useHeading     (() -> driverJoystick.getPOV() != -1)
               .isFieldRelative(() -> driverJoystick.getRawAxis(2) < 0.2) //left trigger
               .isLockIn       (() -> driverJoystick.getRawAxis(3) > 0.2) //right trigger
               .isZeroOdometry (() -> zeroButton.getAsBoolean())
               .isOpenLoop     (() -> !driverJoystick.getRawButton(6)); //right bumper
    driveTrain.register(driverJoystick);

    driveTrain.setDefaultCommand(peaccyDrive);
    sushi.setDefaultCommand(sushi.index());
    climber.setDefaultCommand(climber.rest());
    wrist.setDefaultCommand(wrist.goToSetpoint(WristSetpoints.REST));
    elevator.setDefaultCommand(elevator.goToSetpoint(ElevatorSetpoints.REST));
    chute.setDefaultCommand(chute.rest());

    new ButtonMap(operatorJoystick).map(operatorsMap);
    new ButtonMap(driverJoystick).map(driverMap);
    SmartDashboard.putBoolean("Setpoint Mode", operatorJoystick.getRawButton(setpointModeButton)); //TODO figure out if this will continue to update based on the button input after being declared
  }

  public boolean deployReady()
  {
      if(10 >= wrist.getWristPosition().getDegrees()){
          return false;
      }
      if(sushi.getRearBeamBrake()){
          return false;
      }
      if(elevator.getHeight() <= 0.1){
          return false;
      }
      else{
          return true;      //TODO Ask Shwahilie if returning ends the command early. If it does, this works perfectly fine.      
      }
  }

  public Command getAutonomousCommand() {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");
      return AutoBuilder.resetOdom(path.getStartingHolonomicPose().get()).andThen(AutoBuilder.followPath(path)); //I THINK THE PROBLEMO COULD BE AN AUTOBUILDER ISSUE WITH RETURNING A NULL VALUE
    } catch (Exception e) {
      e.printStackTrace();
    }
    return null;
  }
}