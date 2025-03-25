package frc.robot.commands.drivetrain;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.AllianceFlipUtil;
import frc.robot.subsystems.Swerve;

public class AutoAlign extends Command{
    private final double aligmentDistanceThresholdNoPOV = 1.8; //meters from the center of the reef before auto alignment kicks in
    private final double aligmentDistanceThresholdWithPOV = 2.1; //meters from the center of the reef before auto alignment kicks in
    private final PathConstraints constraints = new PathConstraints(3,3,Units.degreesToRadians(360), Units.degreesToRadians(360));

    private final Swerve swerve;
    private final Supplier<Integer> operatorPOV;

    private Command pathfindingCommand = null;
    private Pose2d targetPose = null;

    public AutoAlign(Swerve swerve, Supplier<Integer> operatorPOV) {
        this.swerve = swerve;
        this.operatorPOV = operatorPOV;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        pathfindingCommand = null;
        targetPose = null;
    }

    @Override
    public void execute() {
        var robotPose = swerve.getPose();
        var reefCenterDist = reefCenter.getDistance(robotPose.getTranslation());

        var POV = operatorPOV.get();
        var hasPov = POV % 45 == 0;

        //don't activate the auto align till we're close enough.
        if(reefCenterDist > (hasPov ? aligmentDistanceThresholdWithPOV : aligmentDistanceThresholdNoPOV)) {
            return;
        }


        boolean needsNewPathfindingCommand = pathfindingCommand == null;
        //always recalculate target pose if the operator is overriding
        if(hasPov) {
            Pose2d newTargetPose;
            if(reefCenterDist > 1.6) newTargetPose = getNearestPlacePosition(robotPose, getPOVBranchSelection(POV));
            else newTargetPose = getNearestPlacePosition(robotPose, getFacePlacePositionsFromPOV(POV));

            if(newTargetPose != targetPose) {
                needsNewPathfindingCommand = true;
                targetPose = newTargetPose;
            }
        } else if (targetPose == null) {
            targetPose = getNearestPlacePosition(robotPose, getAllianePlacePositions());
            needsNewPathfindingCommand = true;
        }

        if(needsNewPathfindingCommand) {
            if(pathfindingCommand != null) pathfindingCommand.cancel();
            pathfindingCommand = AutoBuilder.pathfindToPose(targetPose, constraints);
            pathfindingCommand.schedule();
        }
    }

    public Rotation2d getTargetDrivetrainAngleToPlace() {
        var robotPose = swerve.getPose();
        var angleFromReef = reefCenter.minus(robotPose.getTranslation()).getAngle().getDegrees();
        return Rotation2d.fromDegrees(Math.round(angleFromReef/60)*60);
    }

    public Rotation2d getTargetDrivetrainAngleToIntake() {
        var robotPose = swerve.getPose();
        return robotPose.getY() < 4 ? Rotation2d.fromDegrees(55) : Rotation2d.fromDegrees(-55);
    }

    public boolean aligning() {
        return pathfindingCommand != null;
    }

    /**
     * Locations of each reef branch, as measured by the cameras on the actual field.
     * This is the fudge factor that accounts for error in AprilTag placement, camera angle, etc.
     *
     * POSITIONING:
     *       7  6
     *      ......
     *   8 /      \ 5
     *  9 /        \ 4
     * 10 \        / 3
     *  11 \      / 2
     *      ``````
     *       0  1
     *  --DRIVER STATION SIDE--
     */
    public static final Pose2d[] placePositionsBlue = new Pose2d[]{
        new Pose2d(3.137,4.190,new Rotation2d(0)),
        new Pose2d(3.137,3.860,new Rotation2d(0)),
        new Pose2d(3.666,2.949,new Rotation2d(60)),
        new Pose2d(3.959,2.790,new Rotation2d(60)),
        new Pose2d(5.008,2.788,new Rotation2d(120)),
        new Pose2d(5.295,2.958,new Rotation2d(120)),
        new Pose2d(5.824,3.856,new Rotation2d(180)),
        new Pose2d(5.823,4.191,new Rotation2d(180)),
        new Pose2d(5.300,5.100,new Rotation2d(-120)),
        new Pose2d(5.014,5.236,new Rotation2d(-120)),
        new Pose2d(3.956,5.261,new Rotation2d(-60)),
        new Pose2d(3.673,5.092,new Rotation2d(-60)),
    };

    public static final Pose2d[] placePositionsRed = new Pose2d[]{
        new Pose2d(3.137,4.190,new Rotation2d(0)),
        new Pose2d(3.137,3.860,new Rotation2d(0)),
        new Pose2d(3.666,2.949,new Rotation2d(60)),
        new Pose2d(3.959,2.790,new Rotation2d(60)),
        new Pose2d(5.008,2.788,new Rotation2d(120)),
        new Pose2d(5.295,2.958,new Rotation2d(120)),
        new Pose2d(5.824,3.856,new Rotation2d(180)),
        new Pose2d(5.823,4.191,new Rotation2d(180)),
        new Pose2d(5.300,5.100,new Rotation2d(-120)),
        new Pose2d(5.014,5.236,new Rotation2d(-120)),
        new Pose2d(3.956,5.261,new Rotation2d(-60)),
        new Pose2d(3.673,5.092,new Rotation2d(-60)),
    };

    public static final Translation2d reefCenter = new Translation2d(4.5,4);

    public Pose2d getNearestPlacePosition(Pose2d robotPose, Pose2d[] placePositions) {
        // var placePositions = AllianceFlipUtil.shouldFlip() ? placePositionsRed : placePositionsBlue;
        double bestDistance = robotPose.getTranslation().getDistance(placePositions[0].getTranslation());
        Pose2d bestPose = placePositions[0];

        for(Pose2d i : placePositions){
            var distance = robotPose.getTranslation().getDistance(i.getTranslation());
            if(distance < bestDistance){
                bestPose = i;
                bestDistance = distance;
            }
        }

        return bestPose;
    }

    public Pose2d[] getAllianePlacePositions() {
        return AllianceFlipUtil.shouldFlip() ? placePositionsRed : placePositionsBlue;
    }

    /**
     * get place positions for a face going counter-clockwise around the reef starting from the face closest to the driver station
     * @param face
     * @return
     */
    public Pose2d[] getFacePlacePositions(int face) {
        var placePositions = getAllianePlacePositions();
        if(face < 0 || face > 7) return placePositions;
        var res = new Pose2d[2];
        res[0] = placePositions[face * 2];
        res[1] = placePositions[(face * 2) + 1];
        return res;
    }

    public Pose2d[] getFacePlacePositionsFromPOV(int POV) {
        if(POV % 45 != 0) return getAllianePlacePositions();
        return getFacePlacePositions((180-POV)/45);
    }

    /**
     * Narrow which branches can be driven to based on POV angle.
     * For example, if the POV hat is pressed to the left, only the left branches on each face are allowed
     * (the side faces wouldn't be affected)
     * @param POVAngle
     * @return
     */
    public Pose2d[] getPOVBranchSelection(int POVAngle) {
        int[] allowedFaces;
        switch(POVAngle){
            case 0: //forward
                allowedFaces = new int[]{0,1,3,5,6,7,8,10};
                break;
            case 45: //diagonal forward and right
                allowedFaces = new int[]{1,3,4,5,6,8,10,11};
                break;
            case 90: //right
                allowedFaces = new int[]{1,3,4,6,8,11};
                break;
            case 135: //diagonal backward and right
                allowedFaces = new int[]{1,2,3,4,6,8,9,11};
                break;
            case 180: //POV backward
                allowedFaces = new int[]{0,1,2,4,6,7,9,11};
                break;
            case 225: //diagonal backward and left
                allowedFaces = new int[]{0,2,4,5,7,9,10,11};
                break;
            case 270: //left
                allowedFaces = new int[]{0,2,5,7,9,10};
                break;
            case 315: //diagonal forward and left
                allowedFaces = new int[]{0,2,3,5,7,8,9,10};
                break;
            default:
                return getAllianePlacePositions();
        }

        var res = new Pose2d[allowedFaces.length];
        var placePositions = getAllianePlacePositions();
        for(int i = 0; i < allowedFaces.length; i++){
            res[i] = placePositions[allowedFaces[i]];
        }

        return res;
    }
}
