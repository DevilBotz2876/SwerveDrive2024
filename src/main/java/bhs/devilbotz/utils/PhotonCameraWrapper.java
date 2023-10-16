package bhs.devilbotz.utils;

import bhs.devilbotz.Constants.VisionConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import java.io.IOException;
import java.util.Optional;

/**
 * Wrapper for PhotonCamera and PhotonPoseEstimator
 *
 * <p>PhotonCamera is a wrapper for the PhotonVision camera. PhotonPoseEstimator is a wrapper for
 * the PhotonVision pose estimator. This class is a wrapper for both of those classes. It is
 * responsible for initializing the PhotonCamera and PhotonPoseEstimator, and providing a method to
 * get an EstimatedRobotPose from the PhotonPoseEstimator.
 *
 * @see PhotonCamera
 * @see PhotonPoseEstimator
 * @see EstimatedRobotPose
 * @see Pose2d
 */
public class PhotonCameraWrapper {
    private final PhotonCamera photonCamera;
    private PhotonPoseEstimator photonPoseEstimator;

    /**
     * Initialize the PhotonCamera and PhotonPoseEstimator
     */
    public PhotonCameraWrapper() {
        // Change the name of your camera here to whatever it is in the PhotonVision UI.
        photonCamera = new PhotonCamera(VisionConstants.cameraName);

        try {
            // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the field.
            AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            // Create pose estimator
            photonPoseEstimator =
                    new PhotonPoseEstimator(
                            fieldLayout, PoseStrategy.MULTI_TAG_PNP, photonCamera, VisionConstants.robotToCam);
            photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        } catch (IOException e) {
            // The AprilTagFieldLayout failed to load. We won't be able to estimate poses if we don't know
            // where the tags are.
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            photonPoseEstimator = null;
        }
    }

    /**
     * @param prevEstimatedRobotPose The current best guess at robot pose
     * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to create
     * the estimate
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (photonPoseEstimator == null) {
            // The field layout failed to load, so we cannot estimate poses.
            return Optional.empty();
        }
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }
}