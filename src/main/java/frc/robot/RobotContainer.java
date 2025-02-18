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
import frc.robot.commands.drivetrain.PeaccyDrive;
import frc.robot.subsystems.Sushi;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;
import frc.lib.util.ButtonMap;
import frc.lib.util.ButtonMap.Button;
import frc.lib.util.ButtonMap.MultiButton;
import frc.lib.util.ButtonMap.OIEntry;
import frc.robot.subsystems.Wrist.WristSetpoints;
import frc.robot.subsystems.Chute;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;

import static frc.robot.Constants.Climber.climberClimbButton;
import static frc.robot.Constants.Climber.climberDeployButtonLeft;
import static frc.robot.Constants.Climber.climberDeployButtonRight;
import static frc.robot.Constants.Sushi.*;
import static frc.robot.Constants.Wrist.*;


public class RobotContainer {
  /* OI CONSTANTS */

  /* SUBSYSTEMS */
  private final Swerve driveTrain = new Swerve();
  private final Elevator elevator = new Elevator();
  private final Sushi sushi = new Sushi();
  private final Wrist wrist = new Wrist();
  private final Chute chute = new Chute();
  private final Climber climber = new Climber(this::deployReady, chute::hasDropped);

  // private final DriveTrainTuner driveTrainTuneable = new DriveTrainTuner();

  /* OI DEFINITIONS */
  private final Joystick driverJoystick = new Joystick(0);
  private final Joystick operatorJoystick = new Joystick(1);

  private final JoystickButton zeroButton = new JoystickButton(driverJoystick, Constants.OI.zeroOdometry); //for debugging

  private final OIEntry[] operatorSushiMap = new OIEntry[] {
    Button.onHold(sushi.place(), placeButton),
    Button.onHold(sushi.intake(), intakeButton)
  };

  private final OIEntry[] operatorClimberMap = new OIEntry[] {
    MultiButton.onPress(climber.deploy(), climberDeployButtonLeft, climberDeployButtonRight),
    Button.onPress(climber.getToClimbPos(), climberClimbButton),
    // AnyPOV.bindTo(() -> if(true){});
  };

  private final OIEntry[] operatorWristMap = new OIEntry[] {
    MultiButton.onPress(wrist.goToSetpoint(WristSetpoints.REST), restButtonID, setpointModeButton),
    MultiButton.onPress(wrist.goToSetpoint(WristSetpoints.L1), L1ButtonID, setpointModeButton),
    MultiButton.onPress(wrist.goToSetpoint(WristSetpoints.L2L3), L2L3ButtonID, setpointModeButton),
    MultiButton.onPress(wrist.goToSetpoint(WristSetpoints.L4), L4ButtonID, setpointModeButton)
  };

  /* COMMANDS */
  private final PeaccyDrive peaccyDrive = new PeaccyDrive(driveTrain);

  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

  public RobotContainer() {
    configureBindings();
    SmartDashboard.putData("AUTO MODE", autoChooser);
  }

  private void configureBindings() {
    peaccyDrive.withTranslation(() -> -driverJoystick.getRawAxis(Constants.OI.translationAxis))
               .withStrafe     (() -> -driverJoystick.getRawAxis(Constants.OI.strafeAxis))
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

    new ButtonMap(operatorJoystick).map(operatorSushiMap);
    new ButtonMap(operatorJoystick).map(operatorClimberMap);
    new ButtonMap(operatorJoystick).map(operatorWristMap);
    SmartDashboard.putBoolean("Setpoint Mode", operatorJoystick.getRawButton(setpointModeButton)); //TODO figure out if this will continue to update based on the button input after being declared
  }

  public boolean deployReady()
  {
      if(10 >= wrist.getWristPosition().getDegrees()) //TODO figure out what angle is "too far back"- 10 is the placeholder atm
      {
          return false;
      }
      if(sushi.getRearBeamBrake())
      {
          return false;
      }
      if(elevator.getHeight() <= 0.1)
      {
          return false;
      }
      else
      {
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