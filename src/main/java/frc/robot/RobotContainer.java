// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drivetrain.PeaccyDrive;
import frc.robot.subsystems.Sushi;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;
import frc.robot.commands.climber.ClimberDeploy;
import frc.robot.commands.climber.ManualClimb;
import frc.lib.util.ButtonMap;
import frc.lib.util.ButtonMap.Button;
import frc.lib.util.ButtonMap.OIEntry;
import frc.robot.commands.climber.ClimberClimb;
import frc.robot.subsystems.Wrist.WristSetpoints;
import frc.robot.subsystems.Climber;

import static frc.robot.Constants.Climber.climberClimbButton;
import static frc.robot.Constants.Sushi.*;
import static frc.robot.Constants.Wrist.*;


public class RobotContainer {
  /* OI CONSTANTS */
  private final int translationAxis = 5; //forward/backward
  private final int strafeAxis = 4;
  private final int rotationAxis = 0;
  private final int zeroButtonNo = 7;

  /* SENSORS */

  /* SUBSYSTEMS */
  private final Swerve driveTrain = new Swerve();
  private final Sushi sushi = new Sushi();
  private final Wrist wrist = new Wrist();
  private final Climber climber = new Climber();

  // private final DriveTrainTuner driveTrainTuneable = new DriveTrainTuner();

  /* OI DEFINITIONS */
  private final XboxController commandController = new XboxController(1);
  private final Joystick driverController = new Joystick(0);

  private final JoystickButton zeroButton = new JoystickButton(driverController, zeroButtonNo); //for debugging
  private final OIEntry[] operatorMap = new OIEntry[]{
    Button.onPress(sushi.place(), placeButton)
  };


  /* COMMANDS */
  private final PeaccyDrive peaccyDrive = new PeaccyDrive(driveTrain);

  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

  public RobotContainer() {
    configureBindings();
    SmartDashboard.putData("AUTO MODE", autoChooser);
  }

  private void configureBindings() {
    peaccyDrive.withTranslation(() -> -driverController.getRawAxis(translationAxis))
               .withStrafe     (() -> -driverController.getRawAxis(strafeAxis))
               .withRotation   (() -> -driverController.getRawAxis(rotationAxis))
               .withHeading    (() -> (double) -driverController.getPOV())
               .useHeading     (() -> driverController.getPOV() != -1)
               .isFieldRelative(() -> driverController.getRawAxis(2) < 0.2) //left trigger
               .isLockIn       (() -> driverController.getRawAxis(3) > 0.2) //right trigger
               .isZeroOdometry (() -> zeroButton.getAsBoolean())
               .isOpenLoop     (() -> !driverController.getRawButton(6)); //right bumper
               
    driveTrain.register(driverController);
    driveTrain.setDefaultCommand(peaccyDrive);

    sushi.setDefaultCommand(sushi.index());
    climber.setDefaultCommand(climber.rest());

    //Sushi Buttons
    new JoystickButton(commandController, placeButton).whileTrue(sushi.place());
    new JoystickButton(commandController, intakeButton).onTrue(sushi.intake());

    //Wrist Buttons
    new JoystickButton(commandController, restButtonID).onTrue(wrist.goToSetpoint(WristSetpoints.REST));
    new JoystickButton(commandController, L1ButtonID).onTrue(wrist.goToSetpoint(WristSetpoints.L1));
    new JoystickButton(commandController, L2L3ButtonID).onTrue(wrist.goToSetpoint(WristSetpoints.L2L3));
    new JoystickButton(commandController, L4ButtonID).onTrue(wrist.goToSetpoint(WristSetpoints.L4));

    //ClimberDeploy
    JoystickButton deployButtonRight = new JoystickButton(commandController, Constants.Climber.climberDeployButtonRight);
    new JoystickButton(commandController, Constants.Climber.climberDeployButtonLeft).onTrue(climberDeploy.deploy(deployButtonRight.getAsBoolean()));

    //ClimberClimbPos
    new JoystickButton(commandController, Constants.Climber.climberClimbButton).onTrue(climberClimb.getToClimbPos());
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