package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

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

    // get the first result from the camera
    List<PhotonPipelineResult> results = mCamera.getAllUnreadResults();

    if (results.isEmpty()) {
      return Optional.empty();
    }

    var result = results.get(0);

    visionEst = mPoseEstimator.estimateCoprocMultiTagPose(result);
    if (visionEst.isEmpty()) {
      visionEst = mPoseEstimator.estimateLowestAmbiguityPose(result);
    }

    SmartDashboard.putBoolean("Camera Has Result?", visionEst.isPresent());

    return visionEst;
  }
}
