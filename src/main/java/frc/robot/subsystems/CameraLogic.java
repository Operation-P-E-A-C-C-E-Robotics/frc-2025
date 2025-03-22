package frc.robot.subsystems;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.vision.ApriltagCamera.VisionResults;
import frc.robot.RobotContainer;
import edu.wpi.first.apriltag.AprilTag.*;


public class CameraLogic extends SubsystemBase 
{
    
    public CameraLogic() {}

    @Override
    public void periodic()
    {
    final Optional<EstimatedRobotPose> optionalEstimatedPoseRight = RobotContainer.photonPoseEstimatorRight.update(RobotContainer.camera.getLatestResult());
    final Optional<EstimatedRobotPose> optionalEstimatedPoseLeft = RobotContainer.photonPoseEstimatorLeft.update(RobotContainer.camera.getLatestResult());
    VisionResults vizzies;
    final SwerveDrivePoseEstimator swervePoseEstimator;
      if(optionalEstimatedPoseLeft.isPresent()) 
      {
        final EstimatedRobotPose estimatedPoseLeft = optionalEstimatedPoseLeft.get();   
        //swervePoseEstimator.addVisionMeasurement(estimatedPoseLeft., estimatedPoseLeft.timestampSeconds);
      }
      final EstimatedRobotPose estimatedPoseRight = optionalEstimatedPoseRight.get();
    }
}
