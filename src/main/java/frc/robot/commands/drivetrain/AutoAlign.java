package frc.robot.commands.drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.motion.Trajectory;
import frc.lib.swerve.PeaccyRequest;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AutoAlign extends Command{
    private final Swerve swerve;
    private final Supplier<Pose2d> targetPose;
    private final PeaccyRequest driveRequest;

    private Trajectory driveTrajectory = new Trajectory(new TrapezoidProfile.State(0, 0));
    private final Timer trajectoryTimer = new Timer();

    //Angle of the robot to translate (0 degrees = away from the alliance station)
    Translation2d trajectoryDeltaTranslation = new Translation2d();
    Rotation2d targetDirection = new Rotation2d();
    double targetDistance = 0; //distance the robot needs to translate
        
    public AutoAlign(Swerve swerve, Supplier<Pose2d> targetPose) {
        this.swerve = swerve;
        this.targetPose = targetPose;

        driveRequest  = new PeaccyRequest(
            Constants.Swerve.autoHeadingMaxVelocity, 
            Constants.Swerve.autoHeadingMaxAcceleration,
            Constants.Swerve.lockHeadingMaxVelocity,
            Constants.Swerve.lockHeadingMaxAcceleration,
            Constants.Swerve.teleopLinearMultiplier,
            Constants.Swerve.autoHeadingKP, 
            Constants.Swerve.autoHeadingKV, 
            Constants.Swerve.autoHeadingKA, 
            Constants.Swerve.lockHeadingKP,
            swerve::getChassisSpeeds, 
            swerve::getTotalDriveCurrent, 
            Constants.Swerve.softHeadingCurrentLimit,
            swerve
        ).withRotationalDeadband(Constants.Swerve.teleopAngularVelocityDeadband)
        .withSoftHoldHeading(Constants.Swerve.useSoftHoldHeading)
        .withPositionCorrectionIterations(Constants.Swerve.teleopPositionCorrectionIters);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        generateTrajectory();
    }

    @Override
    public void execute() {

        //regenerate trajectory if distance between trajectory end point and target is greater than a threshold
        if(shouldRegenTrajectory()) generateTrajectory();

        Translation2d translationError = getErrorTranslation();
        //if the robot is far away from the target, use the motion profiling to align
        if(shouldUseTrajectory()) {
            var time = trajectoryTimer.get();
            var profiledTargetDistance = driveTrajectory.calculate(time).position;
            translationError = new Translation2d(profiledTargetDistance, targetDirection);
        }

        translationError = translationError.times(Constants.Swerve.autoAlignKP);
        swerve.drive(driveRequest.withVelocityX(translationError.getX())
                                 .withVelocityY(translationError.getY())
                                 .withHeading(targetPose.get().getRotation().getRadians())
                                 .withHoldHeading(true)
                                 .withIsOpenLoop(true));
    }
    private void generateTrajectory() {
        trajectoryDeltaTranslation = getErrorTranslation();
        targetDirection = trajectoryDeltaTranslation.getAngle();
        targetDistance = trajectoryDeltaTranslation.getNorm();

        driveTrajectory = new Trajectory(new TrapezoidProfile.State(0, 0), new TrapezoidProfile.State(targetDistance, 0));
        trajectoryTimer.reset();
        trajectoryTimer.start();
    }

    private Translation2d getErrorTranslation() {
        return swerve.getPose().getTranslation().minus(targetPose.get().getTranslation());
    }

    private boolean shouldUseTrajectory() {
        return swerve.getPose().getTranslation().getDistance(targetPose.get().getTranslation()) > Constants.Swerve.autoAlignTrajectoryTolerance;
    }

    private boolean shouldRegenTrajectory() {
        return trajectoryDeltaTranslation.getDistance(targetPose.get().getTranslation()) > Constants.Swerve.autoAlignTrajectoryTolerance;
    }
}
