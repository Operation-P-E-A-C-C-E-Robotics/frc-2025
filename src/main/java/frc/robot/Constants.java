// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.function.DoubleFunction;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.lib.swerve.SwerveDescription.CANIDs;
import frc.lib.swerve.SwerveDescription.Dimensions;
import frc.lib.swerve.SwerveDescription.EncoderOffsets;
import frc.lib.swerve.SwerveDescription.Gearing;
import frc.lib.swerve.SwerveDescription.Inversion;
import frc.lib.swerve.SwerveDescription.Physics;
import frc.lib.swerve.SwerveDescription.PidGains;
import frc.lib.util.JoystickCurves;

public final class Constants {
  public static final double period = 0.01; //loop time

  public static final class InputIDS{
    //Driver
    //Operator
    public static int sushiIntakeButton = 6;
  }

  public static final class Elevator {
    public static int elevatorMasterID  = 0;//CAN IDS
    public static int elevatorFollowerID = 0;//CAN IDS
    public static int upperLimitSwitchID = 0;//pwm port ID, labled DIO
    public static int lowerLimitSwitchID = 0;//pwn Port ID, labled DIO
    public static double spoolCircumference = Units.inchesToMeters(2) * Math.PI;



    public static TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    static {
      motorConfig.Slot0.withGravityType(GravityTypeValue.Elevator_Static)
                        .withKP(0)
                        .withKI(0)
                        .withKD(0)
                        .withKS(0)
                        .withKV(0)
                        .withKA(0);

      motorConfig.MotionMagic.withMotionMagicAcceleration(0)
                            .withMotionMagicCruiseVelocity(0)
                            .withMotionMagicJerk(0)
                            .withMotionMagicExpo_kA(0)
                            .withMotionMagicExpo_kV(0);

      // motorConfig.Feedback.withSensorToMechanismRatio(1); TODO

      motorConfig.CurrentLimits.withStatorCurrentLimit(40)
                                .withStatorCurrentLimitEnable(true);

      motorConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive)
                             .withNeutralMode(NeutralModeValue.Brake);

      // TODO enable once limit switches are on elevator. Could cause issues if not connected.
      // motorConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
      motorConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0;

      // motorConfig.HardwareLimitSwitch.ReverseLimitEnable = true;
      // motorConfig.HardwareLimitSwitch.ForwardLimitEnable = true;
     }
  }
    //Sushi
  public static final class Sushi {
    public static final int sushiMainID = 90;
    public static final int frontBeamBreakID = 0;
    public static final int backBeamBreakID = 0;
    public static final double wheelCircumference = Units.inchesToMeters(2) * Math.PI; // 10 cm diameter wheel
    public static final double gearRatio = 10.0;         // Motor:Wheel
    public static final int intakeButton = 90;        // m/s
    public static final int placeButton = 90;    // m/s^2


    public static TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    static {
      motorConfig.Slot0.withKP(0)
                        .withKI(0)
                        .withKD(0)
                        .withKS(0)
                        .withKV(0)
                        .withKA(0);

      motorConfig.CurrentLimits.withStatorCurrentLimit(40)
                               .withStatorCurrentLimitEnable(true);

      motorConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive)
                               .withNeutralMode(NeutralModeValue.Brake);
    }
  }

 ///WRIST
 public static final class Wrist
 {
   public static final int mainMotorID = 0; //TODO set motor can id
   // public static final int spoolCircumference = 0; //TODO

    public static final int restButtonID = 0;
    public static final int L1ButtonID = 1;//TODO: Set this button to something
    public static final int L2L3ButtonID = 2; 
    public static final int L4ButtonID = 3;
    public static final int setpointModeButton = 100; //TODO: Set this button to something
   public static TalonFXConfiguration motorConfig = new TalonFXConfiguration();
   static {
    motorConfig.Slot0.withGravityType(GravityTypeValue.Elevator_Static)
                       .withKP(0)
                       .withKI(0)
                       .withKD(0)
                       .withKS(0)
                       .withKV(0)
                       .withKA(0);
     
     motorConfig.MotionMagic.withMotionMagicAcceleration(0)
                           .withMotionMagicCruiseVelocity(0)
                           .withMotionMagicJerk(0)
                           .withMotionMagicExpo_kA(0)
                           .withMotionMagicExpo_kV(0);

     // motorConfig.Feedback.withSensorToMechanismRatio(1); TODO

     motorConfig.CurrentLimits.withStatorCurrentLimit(40)
                               .withStatorCurrentLimitEnable(true);
     
     motorConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive)
                            .withNeutralMode(NeutralModeValue.Brake);

      // TODO enable once limit switches are on wrist. Could cause issues if not connected.
      // motorConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
      motorConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0;

      // motorConfig.HardwareLimitSwitch.ReverseLimitEnable = true;
      // motorConfig.HardwareLimitSwitch.ForwardLimitEnable = true;
    }
 }

 ///Chute
  public static final class Chute {
    public static final int leftMotorID = 16; //TODO set motor can id
    public static final int rightMotorID = 28;
    public static final int intakeID = 0;//TODO
    public static final int unjamID = 0;
    public static final double intakeSpeed = 0.6; //For now
    public static final double unjamSpeed = -0.8;
}

  public static final class Swerve {
    /* TELEOP */
    //speeds in m/s (probably)
    public static final double teleopLinearMultiplier = 7.0;
    public static final double teleopAngularMultiplier = 7.0;

    //acceleration limits
    public static final double teleopLinearSpeedLimit = 5.0;
	  public static final double teleopLowBatteryLinearSpeedLimit = 2; //more acceleration limit when battery is low
    public static final double teleopLinearAngleLimit = 2.0;
    public static final double teleopAngularRateLimit = 3.0;

    //deadband
    public static final double teleopLinearSpeedDeadband = 0.15;
    public static final double teleopAngularVelocityDeadband = 0.2;
    public static final double teleopDeadbandDebounceTime = 0.1;

    public static final DoubleFunction <Double> teleopLinearSpeedCurve = (double linearSpeed) -> JoystickCurves.herraFCurve(linearSpeed, 6, 4.5); //a nice gentle curve which is Peaccy's (me!!) favorite :)
    public static final DoubleFunction <Double> teleopAngularVelocityCurve = (double angularVelocity) -> JoystickCurves.powerCurve(angularVelocity, 2); //TODO decide if the driver (me) wants a curve on this or not.

    //number of loops to keep track of position correction for (so multiply by 20ms to get the duration the correction is considering)
    public static final int teleopPositionCorrectionIters = 0;


    /* CTRE SWERVE CONSTANTS */
    public static final Dimensions dimensions = new Dimensions(Units.inchesToMeters(18.75), Units.inchesToMeters(18.75));

    //module 0: front right -> rear left
    //module 1: back right -> front left
    //module 3: front left -> back right
    //module 2: back left -> front right

    public static final CANIDs frontLeftIDs =   new CANIDs(7,   5,    6); //module 1
    public static final CANIDs frontRightIDs =   new CANIDs(10, 8, 9); //module 2
    public static final CANIDs rearLeftIDs =    new CANIDs(4,  2,   3); //module 0
    public static final CANIDs rearRightIDs =   new CANIDs(13,   11,    12); //module 3

    public static final Gearing gearing = new Gearing(DriveGearRatios.SDSMK4i_L2, ((150.0 / 7.0) / 1.0), (3.807/2), 0);

    public static final EncoderOffsets offsets = new EncoderOffsets(
      0.042725, //Front left, module 1
      -0.338379, //Front Right, module 2
      -0.062012, //Rear Left, module 0
      0.324463 //Rear Right, module 3
    );

    public static final Inversion inversion = new Inversion(false, true, false, true);

    //inertia only used for simulation
    public static final Physics physics = new Physics(0.05,0.01, Robot.isReal() ? 40 : 800, 7);
    public static final double steerMotorCurrentLimit = Robot.isReal() ? 20 : 120; //TODO: turn this up

    public static final PidGains driveGains = new PidGains(0.2, 0, 0, 0.1, 0); //TODO: tune with final robot
    public static final PidGains angleGains = new PidGains(40, 0, 0, 0, 0); //TODO: tune with final robot

    public static final int pigeonCANId = 0;
    public static final boolean invertSteerMotors = Robot.isReal(); //cant invert in simulation which is dumb.

    /* HEADING CONTROLLER CONSTANTS */
    public static final double autoHeadingKP = 400;
    public static final double lockHeadingKP = 1000;
    public static final double autoHeadingKI = 0.0; //DOES NOTHING LOL
    public static final double autoHeadingKD = 0.0; //ALSO DOES NOTHING LOL
    public static final double autoHeadingKV = 0.7;
    public static final double autoHeadingKA = 0.02;
    public static final double autoHeadingMaxVelocity = 3; //deg/s (i think)
    public static final double autoHeadingMaxAcceleration = 10; //deg/s^2
    public static final double lockHeadingMaxVelocity = 6;
    public static final double lockHeadingMaxAcceleration = 20;
    public static final boolean useSoftHoldHeading = false;
    public static final double softHeadingCurrentLimit = 30;

    public static final double autoAlignKP = 0;
    public static final double autoAlignKV = 0;
    public static final double autoAlignTrajectoryTolerance = 0.2; //distance from target position to switch to plain PID control

    public static final double aimTolerance = 1; //degrees


    /* PATH FOLLOWING CONSTANTS */
    public static final double pathfollowingMaxVelocity = 3,
                              pathfollowingMaxAcceleration = 3,
                              pathfollowingMaxAngularVelocity = 360,
                              pathfollowingMaxAngularAcceleration = 360;

    public static final double measuredMaxVelocity = 3,
                              measuredMaxAcceleration = 3,
                              measuredMaxAngularVelocity = 360,
                              measuredMaxAngularAcceleration = 360;

    public static final PathConstraints autoMaxSpeed = new PathConstraints(
      pathfollowingMaxVelocity,
      pathfollowingMaxAcceleration,
      pathfollowingMaxAngularVelocity,
      pathfollowingMaxAngularAcceleration
    );

    public static final RobotConfig defaultPathplannerConfig = new RobotConfig(
      Units.lbsToKilograms(75), //robot weight
      6.883, //robot MOI
      new ModuleConfig(
        Units.inchesToMeters(2), //wheel radius //TODO: fudge factor
        5, //Measured max robot speed
        1.2, //wheel Coefficient of Friction
        DCMotor.getFalcon500(1).withReduction(Swerve.gearing.driveRatio),
        Swerve.physics.wheelSlipCurrent, //Drive current limit
        1 //drive motors per module
      ),
      Swerve.dimensions.frontLeft, Swerve.dimensions.frontRight, Swerve.dimensions.rearLeft, Swerve.dimensions.rearRight);
  }

  public static final class Climber {
    public static final int leadClimberMotorID = 90;
    public static final int climberDeployButtonRight = 0; //TODO:
    public static final int climberDeployButtonLeft = 0;
    public static final int climberClimbButton = 0;
                            //followerClimberMotorID = 90;

    public static TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    static {
      motorConfig.Slot0.withGravityType(GravityTypeValue.Elevator_Static)
                        .withKP(0)
                        .withKI(0)
                        .withKD(0)
                        .withKS(0)
                        .withKV(0)
                        .withKA(0);
      
      motorConfig.MotionMagic.withMotionMagicAcceleration(0)
                            .withMotionMagicCruiseVelocity(0)
                            .withMotionMagicJerk(0)
                            .withMotionMagicExpo_kA(0)
                            .withMotionMagicExpo_kV(0);

      // motorConfig.Feedback.withSensorToMechanismRatio(1); TODO:

      motorConfig.CurrentLimits.withStatorCurrentLimit(40)
                                .withStatorCurrentLimitEnable(true);
      
      motorConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive)
                             .withNeutralMode(NeutralModeValue.Brake);
     }
  }

  public static final class ControlSystem {
    public static final int PDPCanId = 1;
    public static final ModuleType PDPModuleType = ModuleType.kRev;
  }

  public static final class Cameras {
    public static final String limelight = "limelight";

    public static final String examplePhotonvisionName = "photonvision";

    public static final double LIMELIGHT_FOCAL_LENGTH = (1*83)/0.32; //as calculated by me: (distance * pixels) / size
  }

  public static final class FieldConstants {
    public static final AprilTagFieldLayout aprilTags;
    static {
      try {
        aprilTags = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      } catch (IOException e) {
        throw new RuntimeException(e);
      }
    }
  }
  //Elevator
  //stolen from 364 :D
  public class DriveGearRatios{
    /** SDS MK3 - 8.16 : 1 */
    public static final double SDSMK3_Standard = (8.16 / 1.0);
    /** SDS MK3 - 6.86 : 1 */
    public static final double SDSMK3_Fast = (6.86 / 1.0);

    /** SDS MK4 - 8.14 : 1 */
    public static final double SDSMK4_L1 = (8.14 / 1.0);
    /** SDS MK4 - 6.75 : 1 */
    public static final double SDSMK4_L2 = (6.75 / 1.0);
    /** SDS MK4 - 6.12 : 1 */
    public static final double SDSMK4_L3 = (6.12 / 1.0);
    /** SDS MK4 - 5.14 : 1 */
    public static final double SDSMK4_L4 = (5.14 / 1.0);
    
    /** SDS MK4i - 8.14 : 1 */
    public static final double SDSMK4i_L1 = (8.14 / 1.0);
    /** SDS MK4i - 6.75 : 1 */
    public static final double SDSMK4i_L2 = (6.75 / 1.0);
    /** SDS MK4i - 6.12 : 1 */
    public static final double SDSMK4i_L3 = (6.12 / 1.0);
}
}
