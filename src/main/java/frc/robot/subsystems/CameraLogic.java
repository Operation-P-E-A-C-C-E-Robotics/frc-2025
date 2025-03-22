package frc.robot.subsystems;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;


public class CameraLogic extends SubsystemBase 
{
    
    public CameraLogic() {}

    // @Override
    // public void periodic()
    // {
    // final Optional<EstimatedRobotPose> optionalEstimatedPoseRight = RobotContainer.photonPoseEstimatorRight.update(RobotContainer.camera.getLatestResult());
    // final Optional<EstimatedRobotPose> optionalEstimatedPoseLeft = RobotContainer.photonPoseEstimatorLeft.update(RobotContainer.camera.getLatestResult());
    // final SwerveDrivePoseEstimator swervePoseEstimator;
    //   if
    //   (optionalEstimatedPoseLeft.isPresent()) {
    //     //final EstimatedRobotPose estimatedPoseLeft = optionalEstimatedPoseLeft.get();   
        
        
    //     //swervePoseEstimator.addVisionMeasurement(estimatedPoseLeft.toString(), estimatedPoseLeft.timestampSeconds);
    //   }
    //   final EstimatedRobotPose estimatedPoseRight = optionalEstimatedPoseRight.get();
    // }
}
