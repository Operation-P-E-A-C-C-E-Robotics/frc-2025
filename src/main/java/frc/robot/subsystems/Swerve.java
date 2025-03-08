package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveDrivetrain.LegacySwerveDriveState;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.safety.Inspiration;
import frc.lib.swerve.PeaccefulSwerve;
import frc.lib.swerve.SwerveDescription;
import frc.lib.swerve.SwerveDescription.PidGains;
import frc.lib.telemetry.SwerveTelemetry;
import frc.lib.util.AllianceFlipUtil;
import frc.lib.vision.ApriltagCamera;
import frc.lib.vision.PeaccyVision;
import frc.robot.Constants;
import frc.robot.FieldConstants;

import static frc.robot.Constants.Swerve.*;

public class Swerve extends SubsystemBase {
    protected final PeaccefulSwerve swerve;

    private final LegacySwerveRequest.ApplyChassisSpeeds autonomousRequest = new LegacySwerveRequest.ApplyChassisSpeeds()
                                                                                        .withDriveRequestType(DriveRequestType.Velocity);
    private final SendableChooser<Pose2d> poseSeedChooser = new SendableChooser<>();

    // private LimelightHelper limelight;

    private static PeaccyVision eyes = new PeaccyVision(
        // new ApriltagCamera.ApriltagLimelight(Constants.Cameras.limelight, 0.1),
        new ApriltagCamera.ApriltagPhotonvision(
            Constants.Cameras.examplePhotonvisionName, 
            new Transform3d(0,0,0, new Rotation3d(0,0,0)), 
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField), 0.001)
    );

    public Swerve() {
        swerve = SwerveDescription.generateDrivetrain(
            dimensions,
            frontLeftIDs,
            frontRightIDs,
            rearLeftIDs,
            rearRightIDs,
            gearing,
            offsets,
            inversion,
            physics,
            driveGains,
            angleGains,
            pigeonCANId,
            invertSteerMotors
        );

        swerve.setSteerCurrentLimit(steerMotorCurrentLimit);

        //pathplanner config
        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config;
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
            config = Constants.Swerve.defaultPathplannerConfig;
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> drive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(1.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(1.0, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    // var alliance = DriverStation.getAlliance();
                    // if (alliance.isPresent()) {
                    //     return alliance.get() == DriverStation.Alliance.Red;
                    // }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );

        //log swerve state data as fast as it comes in
        swerve.registerTelemetry((LegacySwerveDriveState state) -> {
            SwerveTelemetry.updateSwerveState(state, ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getPose().getRotation()), swerve.getPose3d());
        });

        poseSeedChooser.setDefaultOption("zero", new Pose2d());
        poseSeedChooser.addOption("test", new Pose2d(1, 1, new Rotation2d()));
        SmartDashboard.putData("POSE SEED", poseSeedChooser);
        SmartDashboard.putBoolean("seed pose", false);

        System.out.println("DriveTrain Initialized");
    }

    /**
     * make it go.
     * @param request the request to apply to the drivetrain.
     */
    public void drive(LegacySwerveRequest request) {
        swerve.setControl(request);
    }

    /**
     * make it go in auto.
     * @param speeds the chassis speeds to apply to the drivetrain.
     */
    public void drive(ChassisSpeeds speeds) {
        drive(autonomousRequest.withSpeeds(speeds));
    }

    /**
     * the missile knows where it is at all times. it knows this because it knows where it isn't.
     * @return the pose of the robot.
     */
    public Pose2d getPose () {
        var pose = swerve.getState().Pose;
        if (pose == null) return new Pose2d();
        if(AllianceFlipUtil.shouldFlip()) {
            var redAllianceOrigin = new Pose2d(FieldConstants.fieldLength, FieldConstants.fieldWidth, Rotation2d.fromDegrees(180));
            pose = pose.relativeTo(redAllianceOrigin);
        }
        return pose;
    }

    /**
     * this missile even knows how fast it's traveling. it knows this because it knows how fast it isn't traveling.
     * @return the chassis speeds of the robot.
     */
    public ChassisSpeeds getChassisSpeeds() {
        return swerve.getChassisSpeeds();
    }

    /**
     * sometimes, the missile forgets where it is, and it's not even where it's been.
     */
    public void resetOdometry() {
        swerve.seedFieldRelative();
    }

    /**
     * sometimes, we need to tell the missile where it is, and it's not even where it's been.
     * By subtracting where it's been from where it is, or where it's going from where it was, we get
     * where it should be.
     * @param pose the pose to set the robot to.
     */
    public void resetOdometry(Pose2d pose) {
        swerve.seedFieldRelative(pose);
    }
    
    /**
     * a workaround for CTRE's moronic zeroing behavior.
     */
    public void zeroFieldCentric() {
        var cachedPose = getPose();
        resetOdometry(AllianceFlipUtil.apply(new Pose2d()));
        resetOdometry();
        resetOdometry(cachedPose);
    }
    
    public PeaccyVision getCameras(){
        return eyes;
    }

    public Rotation3d getGyroAngle() {
        return swerve.getRotation3d();
    }

    public double getTotalDriveCurrent(){
        return swerve.getTotalDriveCurrent();
    }

    /**
     * DO NOT use this in normal operation. For calibration only
     * @param gains the new gains to apply to the drive motors.
     */
    public void updateDriveGains(PidGains gains){
        swerve.applyDriveConfigs(gains);
    }

    /**
     * DO NOT use this in normal operation. For calibration only
     * @param gains the new gains to apply to the steer motors
     */
    public void updateAngleGains(PidGains gains){
        swerve.applySteerConfigs(gains);
    }

    @Override
    public void periodic() {
        if(SmartDashboard.getBoolean("seed pose", false)) {
            var startPose = poseSeedChooser.getSelected();
            resetOdometry(new Pose2d(
                AllianceFlipUtil.apply(startPose.getTranslation()),
                startPose.getRotation()
            ));;
            SmartDashboard.putBoolean("seed pose", false);
        }

        BaseStatusSignal.refreshAll(swerve.getPigeon2().getAccelerationX(), swerve.getPigeon2().getAccelerationY(), swerve.getPigeon2().getAccelerationZ());
        var acceleration = swerve.getPigeon2().getAccelerationX().getValue().magnitude() + swerve.getPigeon2().getAccelerationY().getValue().magnitude() + swerve.getPigeon2().getAccelerationZ().getValue().magnitude();
        eyes.update(getPose(), acceleration, new Translation2d(getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond).getNorm());
        if(eyes.hasUpdated()){
            swerve.addVisionMeasurement(
                eyes.getPose(),
                eyes.getTimestamp(),
                eyes.getStDev()
            );
        }
    }

    @Override
    public void simulationPeriodic() {
        swerve.updateSimState(Constants.period, 12);
    }

    public void register(Joystick j){
        Inspiration.fullPeacce(j);
    }
}

