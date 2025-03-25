// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.io.File;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutomationCommands;
import frc.robot.commands.drivetrain.AutoAlign;
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
import frc.lib.util.ButtonMap.MultiButton;
import org.photonvision.PhotonCamera.*;
import org.photonvision.PhotonPoseEstimator.*;
import frc.lib.vision.ApriltagCamera.*;

public class RobotContainer {
  /* OI CONSTANTS */

  /* SUBSYSTEMS */
  private final Sushi sushi = new Sushi();
  private final Wrist wrist = new Wrist();                                                                                                                      
  private final Elevator elevator = new Elevator(() -> !sushi.getRearBeamBrake(), () -> wrist.getWristPosition().getDegrees() > 60);                                                                               //private String song = "output.chrp"; private static Sushi musicalSushi = new Sushi(); ArrayList<TalonFX> instruments = new ArrayList<TalonFX>(); for(int i = 0; i < 0; i++) {instruments.add(new TalonFX(0));} private Orchestra music = new Orchestra();
  private final Chute chute = new Chute();
  private final Climber climber = new Climber(this::deployReady, chute::hasDropped);
  private final Swerve driveTrain = new Swerve ();



  /* OI DEFINITIONS */
  private final Joystick driverJoystick = new Joystick(0);
  private final Joystick operatorJoystick = new Joystick(1);
  
  private final JoystickButton zeroButton = new JoystickButton(driverJoystick, Constants.OI.zeroOdometry); //for debugging

  //Auto Camera Stuff
  public static AprilTagFieldLayout aprilTagFieldLayout = FieldConstants.AprilTagLayoutType.OFFICIAL.getLayout();  
  public static Transform3d robotToCamLeft = new Transform3d(new Translation3d(Units.inchesToMeters(7.47), Units.inchesToMeters(13.5), Units.inchesToMeters(0.5)), new Rotation3d(0,Units.degreesToRadians(285),0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  public static Transform3d robotToCamRight = new Transform3d(new Translation3d(Units.inchesToMeters(7.35), Units.inchesToMeters(13.5), Units.inchesToMeters(20.5)), new Rotation3d(0,Units.degreesToRadians(281.2),0));
  public static PhotonPoseEstimator photonPoseEstimatorRight = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, robotToCamRight);
  public static PhotonPoseEstimator photonPoseEstimatorLeft = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, robotToCamLeft);  
  /* COMMANDS */
  private final PeaccyDrive peaccyDrive = new PeaccyDrive(driveTrain);
  private final AutoAlign autoAlign = new AutoAlign(driveTrain, () -> operatorJoystick.getPOV());
  private final AutomationCommands automationCommands = new AutomationCommands(driveTrain, elevator, wrist, sushi, chute, autoAlign);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  private final OIEntry[] operatorsMap = new OIEntry[] {
    Button.onHold(sushi.place(() -> (elevator.getHeight() < 5.0) && (elevator.getHeight() > 1.5)), 8),
    Button.onHold(sushi.intake(), 7),
    Button.onHold(chute.intake(), 7),
    Button.onRelease(chute.rest(), 7),
    Button.onHold(chute.unjam().alongWith(sushi.panic()), 5),
    Button.onRelease(chute.rest(), 5),
    
    Button.onHold(automationCommands.l1ElevatorWrist(), 3),
    Button.onHold(automationCommands.l2ElevatorWrist(), 2),
    MultiButton.onHold(automationCommands.l2_5ElevatorWrist(), 9, 2), //
    Button.onHold(automationCommands.l3ElevatorWrist(), 1),
    MultiButton.onHold(automationCommands.l3_5ElevatorWrist(), 9, 1), // TODO PARKER Do The Buttons
    Button.onHold(automationCommands.l4ElevatorWrist(), 4),
    
    MultiButton.onHold(climber.deploy(), 9, 10),
    MultiButton.onHold(chute.dropCommand(), 9, 10),
    
    Button.onPress(elevator.goToSetpoint(ElevatorSetpoints.REST),12),
    Button.onPress(wrist.goToSetpoint(WristSetpoints.REST),12),
    
    JoystickTrigger.onMove(operatorJoystick, elevator.manualInput(() -> -operatorJoystick.getRawAxis(3)*0.3), 3, 0.15),
    JoystickTrigger.onMove(operatorJoystick, wrist.manualInput(() -> -operatorJoystick.getRawAxis(1)*0.3), 1, 0.15),
    JoystickTrigger.onMove(operatorJoystick, climber.manualInput(() -> -operatorJoystick.getRawAxis(0)*0.5), 0, 0.2),
  };
  
  private final OIEntry[] driverMap = new OIEntry[] {
    JoystickTrigger.onMove(driverJoystick, automationCommands.intakeUntilCoralObtained(), 2, 0.5),
    JoystickTrigger.onZero(driverJoystick, sushi.index().alongWith(chute.rest()), 2, 0.4),
    JoystickTrigger.onMove(driverJoystick, sushi.place(() -> (elevator.getHeight() < 5.0) && (elevator.getHeight() > 1.5)), 3, .5),
    
    Button.onHold(automationCommands.l1ElevatorWrist(), 2),
    Button.onHold(automationCommands.l2ElevatorWrist(), 1),
    Button.onHold(automationCommands.l3ElevatorWrist(), 3),
    Button.onHold(automationCommands.l4ElevatorWrist(), 4),
  };
  
  //-------------------------------------------------------------------------------------------------------------------------------------

  public RobotContainer() {
    photonPoseEstimatorLeft.getRobotToCameraTransform();
    configureBindings();
    try {
      PathPlannerPath l1Path = PathPlannerPath.fromPathFile("Drive To Reef");
      PathPlannerPath l1OppositePath = PathPlannerPath.fromPathFile("Drive To Reef Opposite Side");
      PathPlannerPath l1Center = PathPlannerPath.fromPathFile("Drive To Reef Center");
      PathPlannerPath driveOffLinePath = PathPlannerPath.fromPathFile("Drive Off Line");
      Command l1AllianceSideAuto = AutoBuilder.resetOdom(l1Path.getStartingHolonomicPose().get()).andThen(AutoBuilder.followPath(l1Path).raceWith(sushi.index()).andThen(automationCommands.l1ElevatorWrist().alongWith(new WaitCommand(0.5).andThen(sushi.place(() -> true).withTimeout(2), sushi.index())))); //I THINK THE PROBLEMO COULD BE AN AUTOBUILDER ISSUE WITH RETURNING A NULL VALUE
      Command l1OppositeSideAuto = AutoBuilder.resetOdom(l1OppositePath.getStartingHolonomicPose().get()).andThen(AutoBuilder.followPath(l1Path).raceWith(sushi.index()).andThen(automationCommands.l1ElevatorWrist().alongWith(new WaitCommand(0.5).andThen(sushi.place(() -> true).withTimeout(2), sushi.index())))); //I THINK THE PROBLEMO COULD BE AN AUTOBUILDER ISSUE WITH RETURNING A NULL VALUE
      Command l1CenterAuto = AutoBuilder.resetOdom(l1Center.getStartingHolonomicPose().get()).andThen(AutoBuilder.followPath(l1Path).raceWith(sushi.index()).andThen(automationCommands.l1ElevatorWrist().alongWith(new WaitCommand(0.5).andThen(sushi.place(() -> true).withTimeout(2), sushi.index())))); //I THINK THE PROBLEMO COULD BE AN AUTOBUILDER ISSUE WITH RETURNING A NULL VALUE
      Command driveOffLineAuto = AutoBuilder.resetOdom(driveOffLinePath.getStartingHolonomicPose().get()).andThen(AutoBuilder.followPath(driveOffLinePath).raceWith(sushi.index()));
    
      autoChooser.addOption("DO NOTHING", null);
      autoChooser.setDefaultOption("Drive Off Line", driveOffLineAuto);
      autoChooser.addOption("L1 Coral", l1AllianceSideAuto);
      autoChooser.addOption("L1 Opposite Coral", l1OppositeSideAuto);
      autoChooser.addOption("L1 Center Coral", l1CenterAuto);
    } catch (Exception e) {
      e.printStackTrace();
    }
    SmartDashboard.putData("AUTO MODE", autoChooser);
  }

  private void configureBindings() {
    peaccyDrive.withTranslation(() -> (-driverJoystick.getRawAxis(Constants.OI.translationAxis) * 0.7) / (Math.max(elevator.getHeight()/4,1)))
               .withStrafe     (() -> (-driverJoystick.getRawAxis(Constants.OI.strafeAxis) * 0.7) / (Math.max(elevator.getHeight()/4,1)))
               .withRotation   (() -> -driverJoystick.getRawAxis(Constants.OI.rotationAxis))
               .withHeading    (() -> Math.round(driveTrain.getPose().getRotation().getDegrees()/60)*60)
               .useHeading     (() -> driverJoystick.getRawButton(1) || driverJoystick.getRawButton(2) || driverJoystick.getRawButton(3) || driverJoystick.getRawButton(4))
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
  }

  public static AprilTagFieldLayout getAprilTagFieldLayout() {
    return aprilTagFieldLayout;
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
    return autoChooser.getSelected();
  }
}