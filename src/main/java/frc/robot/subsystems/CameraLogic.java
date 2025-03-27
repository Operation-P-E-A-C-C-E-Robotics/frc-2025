package frc.robot.subsystems;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import frc.robot.RobotContainer;

public class CameraLogic extends SubsystemBase {
    private static final Vector<N3> STATE_STDS = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
    private static final Vector<N3> VISION_STDS = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(10));
    
    private final SwerveDriveKinematics driveKinematics;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final PhotonCamera cameraRight;
    private final PhotonCamera cameraLeft;
    // private final PhotonPoseEstimator photonPoseEstimatorRight;
    // private final PhotonPoseEstimator photonPoseEstimatorLeft;
    private final Swerve swerve;

    public CameraLogic(Swerve swerve, SwerveDriveKinematics kinematics) {
        this.swerve = swerve;
        this.driveKinematics = kinematics;
        
        cameraRight = new PhotonCamera("rightCamera");
        cameraLeft = new PhotonCamera("leftCamera");

        poseEstimator = new SwerveDrivePoseEstimator(
            driveKinematics,
            swerve.getGyroAngle().toRotation2d(),
            new SwerveModulePosition[4], // idk internet told me this
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            STATE_STDS,
            VISION_STDS
        );

        Transform3d RIGHT_CAMERA_TO_CENTER = new Transform3d(
            new Translation3d(Units.inchesToMeters(5.760), Units.inchesToMeters(-12.707), Units.inchesToMeters(43.3)),
            new Rotation3d(0, 0, Units.degreesToRadians(-35.895 / 2))
        );

        Transform3d LEFT_CAMERA_TO_CENTER = new Transform3d(
            new Translation3d(Units.inchesToMeters(5.800), Units.inchesToMeters(-8.517), Units.inchesToMeters(43.3)),
            new Rotation3d(0, 0, Units.degreesToRadians(35.895 / 2))
        );

        // AprilTagFieldLayout layout = RobotContainer.getAprilTagFieldLayout();

        // photonPoseEstimatorRight = new PhotonPoseEstimator(
        //     layout,
        //     PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        //     RIGHT_CAMERA_TO_CENTER
        // );

        // photonPoseEstimatorLeft = new PhotonPoseEstimator(
        //     layout,
        //     PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        //     LEFT_CAMERA_TO_CENTER
        // );

        // photonPoseEstimatorRight.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        // photonPoseEstimatorLeft.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void periodic() {

        // Optional<EstimatedRobotPose> rightResult = getEstimatedGlobalPose(photonPoseEstimatorRight, cameraRight);
        // if (rightResult.isPresent()) {
        //     EstimatedRobotPose pose = rightResult.get();
        //     swerve.addVisionMeasurement(
        //         pose.estimatedPose.toPose2d(),
        //         pose.timestampSeconds
        //     );
        // }

        // Optional<EstimatedRobotPose> leftResult = getEstimatedGlobalPose(photonPoseEstimatorLeft, cameraLeft);
        // if (leftResult.isPresent()) {
        //     EstimatedRobotPose pose = leftResult.get();
        //     swerve.addVisionMeasurement(
        //         pose.estimatedPose.toPose2d(),
        //         pose.timestampSeconds
        //     );
        // }
    }
    
    // private Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonPoseEstimator estimator, PhotonCamera camera) {
    //     try {
    //         PhotonPipelineResult latestResult = camera.getLatestResult();
            
    //         return estimator.update(latestResult);
    //     } catch (Exception e) {
    //         return Optional.empty();
    //     }
    // }
    
    // public Pose2d getCurrentPose() {
    //     return poseEstimator.getEstimatedPosition();
    // }
}