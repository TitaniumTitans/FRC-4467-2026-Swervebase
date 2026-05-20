package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import java.util.List;
import java.util.Optional;

public class Vision {
  private final PhotonCamera mCamera;
  private final PhotonPoseEstimator mPoseEstimator;

  // Constructor sets up the vision class on creation
  public Vision() {
    mCamera = new PhotonCamera("camera");
    mPoseEstimator = new PhotonPoseEstimator(
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded),
        new Transform3d());
  }

  // get vision update
  public Optional<EstimatedRobotPose> getVisionUpdate() {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();

    // loop over every result from the camera
    var result = mCamera.getAllUnreadResults().get(0);
    visionEst = mPoseEstimator.estimateCoprocMultiTagPose(result);
    if (visionEst.isEmpty()) {
      visionEst = mPoseEstimator.estimateLowestAmbiguityPose(result);
    }

    return visionEst;
  }
}
