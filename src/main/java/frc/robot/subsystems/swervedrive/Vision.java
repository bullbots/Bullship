package frc.robot.subsystems.swervedrive;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import java.awt.Desktop;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;

/**
 * Example PhotonVision class to aid in the pursuit of accurate odometry. Taken
 * from
 * https://gitlab.com/ironclad_code/ironclad-2024/-/blob/master/src/main/java/frc/robot/vision/Vision.java?ref_type=heads
 */
public class Vision {

  /**
   * April Tag Field Layout of the year.
   */
  public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout
      .loadField(AprilTagFields.k2025ReefscapeAndyMark);
  /**
   * Ambiguity defined as a value between (0,1). Used in
   * {@link Vision#filterPose}.
   */
  private final double maximumAmbiguity = 0.25;
  /**
   * Photon Vision Simulation
   */
  public VisionSystemSim visionSim;
  /**
   * Count of times that the odom thinks we're more than 10meters away from the
   * april tag.
   */
  private double longDistangePoseEstimationCount = 0;
  /**
   * Current pose from the pose estimator using wheel odometry.
   */
  private Supplier<Pose2d> currentPose;
  /**
   * Field from {@link swervelib.SwerveDrive#field}
   */
  private Field2d field2d;
  /**
   * Track whether we've seen an AprilTag and reset odometry yet
   */
  private boolean hasResetOdometryFromVision = false;
  /**
   * Track whether we've printed the waiting message
   */
  private boolean hasLoggedWaitingMessage = false;

  /**
   * Constructor for the Vision class.
   *
   * @param currentPose Current pose supplier, should reference
   *                    {@link SwerveDrive#getPose()}
   * @param field       Current field, should be {@link SwerveDrive#field}
   */
  public Vision(Supplier<Pose2d> currentPose, Field2d field) {
    this.currentPose = currentPose;
    this.field2d = field;

    if (Robot.isSimulation()) {
      try {
        visionSim = new VisionSystemSim("Vision");
        visionSim.addAprilTags(fieldLayout);

        for (Cameras c : Cameras.values()) {
          c.addToVisionSim(visionSim);
        }

        openSimCameraViews();
      } catch (Exception e) {
        System.err.println("Warning: PhotonVision simulation initialization failed: " + e.getMessage());
        System.err.println("Vision simulation will be disabled, but real robot functionality is unaffected.");
        visionSim = null;
      }
    }

    // Flush stale PhotonVision results that accumulated before robot code started
    // This prevents reading old buffered data when robot code restarts
    flushStaleResults();
  }

  /**
   * Flush any stale results from PhotonVision cameras.
   * Call this on startup to clear buffered data from before robot code started.
   */
  private void flushStaleResults() {

    for (Cameras camera : Cameras.values()) {
      // Call getAllUnreadResults() to clear the buffer, discard the results
      var staleResults = camera.camera.getAllUnreadResults();

      // Also clear the camera's internal results list
      camera.resultsList.clear();
    }

  }

  /**
   * Calculates a target pose relative to an AprilTag on the field.
   *
   * @param aprilTag    The ID of the AprilTag.
   * @param robotOffset The offset {@link Transform2d} of the robot to apply to
   *                    the pose for the robot to position
   *                    itself correctly.
   * @return The target pose of the AprilTag.
   */
  public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset) {
    Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(aprilTag);
    if (aprilTagPose3d.isPresent()) {
      return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
    } else {
      throw new RuntimeException("Cannot get AprilTag " + aprilTag + " from field " + fieldLayout.toString());
    }

  }

  /**
   * Reset the vision odometry flag. Call this when starting teleop to ensure
   * AprilTag detection can reset odometry.
   */
  public void resetVisionOdometryFlag() {
    hasResetOdometryFromVision = false;
    hasLoggedWaitingMessage = false;
    System.out.println("[Vision] Reset odometry flag - ready to detect AprilTags");

    // Print camera status for diagnostics
    if (Robot.isReal()) {
      printCameraStatus();
    }
  }

  /**
   * Print the status of all cameras for diagnostic purposes.
   */
  public void printCameraStatus() {
    System.out.println("[Vision] Camera Status Check:");
    for (Cameras camera : Cameras.values()) {
      try {
        boolean isConnected = camera.camera.isConnected();
        System.out.println("  - " + camera.name() + ": " + (isConnected ? "CONNECTED" : "DISCONNECTED"));

        if (!isConnected) {
          System.err.println("    WARNING: " + camera.name() + " is not connected to PhotonVision!");
        }
      } catch (Exception e) {
        System.err.println("  - " + camera.name() + ": ERROR checking connection - " + e.getMessage());
      }
    }
  }

  /**
   * Update the pose estimation inside of {@link SwerveDrive} with all of the
   * given poses.
   *
   * @param swerveDrive {@link SwerveDrive} instance.
   */
  public void updatePoseEstimation(SwerveDrive swerveDrive) {
    if (SwerveDriveTelemetry.isSimulation && visionSim != null) {
      /*
       * In simulation, update vision simulation with the true robot pose.
       * Don't force-reset odometry as it disrupts velocity calculations.
       */
      try {
        var simPose = swerveDrive.getSimulationDriveTrainPose();
        if (simPose.isPresent()) {
          // Update vision simulation with the true pose
          visionSim.update(simPose.get());
        } else {
          // Fallback: use odometry pose for vision simulation if simulation pose isn't
          // available
          visionSim.update(swerveDrive.getPose());
        }
      } catch (Exception e) {
        // Silently ignore simulation errors to prevent spam
      }
    }

    // Debug: Print waiting message only once
    if (!hasResetOdometryFromVision && !hasLoggedWaitingMessage) {
      System.out.println("[Vision] Waiting for AprilTag detection to reset odometry...");
      hasLoggedWaitingMessage = true;
    }

    // In simulation, disable vision odometry corrections entirely
    // Simulated wheel encoders are perfectly accurate, vision just adds instability
    // Vision estimates are still calculated and visible in Glass for debugging
    if (Robot.isSimulation()) {
      for (Cameras camera : Cameras.values()) {
        getEstimatedGlobalPose(camera); // Calculate for visualization only
      }
      return; // Don't apply any vision measurements to odometry
    }

    // Real robot: Apply vision corrections normally
    for (Cameras camera : Cameras.values()) {
      Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);
      // Only log when pose is actually present to reduce console spam
      if (poseEst.isPresent()) {
        var pose = poseEst.get();

        // First time seeing a tag: reset odometry to immediately snap to correct position
        if (!hasResetOdometryFromVision) {
          System.out.println("[Vision] First AprilTag detected! Resetting odometry to: " + pose.estimatedPose.toPose2d());
          swerveDrive.resetOdometry(pose.estimatedPose.toPose2d());
          hasResetOdometryFromVision = true;
        } else {
          // Subsequent detections: blend vision with odometry using configured standard deviations
          swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(),
              pose.timestampSeconds,
              camera.curStdDevs);
        }
      }
    }

  }

  /**
   * Generates the estimated robot pose. Returns empty if:
   * <ul>
   * <li>No Pose Estimates could be generated</li>
   * <li>The generated pose estimate was considered not accurate</li>
   * </ul>
   *
   * @param camera Camera to get pose estimate from
   * @return an {@link EstimatedRobotPose} with an estimated pose, timestamp, and
   *         targets used to create the estimate
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Cameras camera) {
    Optional<EstimatedRobotPose> poseEst = camera.getEstimatedGlobalPose();
    if (Robot.isSimulation() && visionSim != null) {
      try {
        Field2d debugField = visionSim.getDebugField();
        // Uncomment to enable outputting of vision targets in sim.
        poseEst.ifPresentOrElse(
            est -> debugField
                .getObject("VisionEstimation")
                .setPose(est.estimatedPose.toPose2d()),
            () -> {
              debugField.getObject("VisionEstimation").setPoses();
            });
      } catch (Exception e) {
        // Silently ignore simulation errors
      }
    }
    return poseEst;
  }

  /**
   * Filter pose via the ambiguity and find best estimate between all of the
   * camera's throwing out distances more than
   * 10m for a short amount of time.
   *
   * @param pose Estimated robot pose.
   * @return Could be empty if there isn't a good reading.
   */
  @Deprecated(since = "2024", forRemoval = true)
  private Optional<EstimatedRobotPose> filterPose(Optional<EstimatedRobotPose> pose) {
    if (pose.isPresent()) {
      double bestTargetAmbiguity = 1; // 1 is max ambiguity
      for (PhotonTrackedTarget target : pose.get().targetsUsed) {
        double ambiguity = target.getPoseAmbiguity();
        if (ambiguity != -1 && ambiguity < bestTargetAmbiguity) {
          bestTargetAmbiguity = ambiguity;
        }
      }
      // ambiguity to high dont use estimate
      if (bestTargetAmbiguity > maximumAmbiguity) {
        return Optional.empty();
      }

      // est pose is very far from recorded robot pose
      if (PhotonUtils.getDistanceToPose(currentPose.get(), pose.get().estimatedPose.toPose2d()) > 1) {
        longDistangePoseEstimationCount++;

        // if it calculates that were 10 meter away for more than 10 times in a row its
        // probably right
        if (longDistangePoseEstimationCount < 10) {
          return Optional.empty();
        }
      } else {
        longDistangePoseEstimationCount = 0;
      }
      return pose;
    }
    return Optional.empty();
  }

  /**
   * Get distance of the robot from the AprilTag pose.
   *
   * @param id AprilTag ID
   * @return Distance
   */
  public double getDistanceFromAprilTag(int id) {
    Optional<Pose3d> tag = fieldLayout.getTagPose(id);
    return tag.map(pose3d -> PhotonUtils.getDistanceToPose(currentPose.get(), pose3d.toPose2d())).orElse(-1.0);
  }

  /**
   * Get tracked target from a camera of AprilTagID
   *
   * @param id     AprilTag ID
   * @param camera Camera to check.
   * @return Tracked target.
   */
  public PhotonTrackedTarget getTargetFromId(int id, Cameras camera) {
    PhotonTrackedTarget target = null;
    for (PhotonPipelineResult result : camera.resultsList) {
      if (result.hasTargets()) {
        for (PhotonTrackedTarget i : result.getTargets()) {
          if (i.getFiducialId() == id) {
            return i;
          }
        }
      }
    }
    return target;

  }

  /**
   * Vision simulation.
   *
   * @return Vision Simulation
   */
  public VisionSystemSim getVisionSim() {
    return visionSim;
  }

  /**
   * Open up the photon vision camera streams on the localhost, assumes running
   * photon vision on localhost.
   */
  private void openSimCameraViews() {
    if (Desktop.isDesktopSupported() && Desktop.getDesktop().isSupported(Desktop.Action.BROWSE)) {
      // try
      // {
      // Desktop.getDesktop().browse(new URI("http://localhost:1182/"));
      // Desktop.getDesktop().browse(new URI("http://localhost:1184/"));
      // Desktop.getDesktop().browse(new URI("http://localhost:1186/"));
      // } catch (IOException | URISyntaxException e)
      // {
      // e.printStackTrace();
      // }
    }
  }

  /**
   * Update the {@link Field2d} to include tracked targets/
   */
  public void updateVisionField() {

    List<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>();
    for (Cameras c : Cameras.values()) {
      if (!c.resultsList.isEmpty()) {
        PhotonPipelineResult latest = c.resultsList.get(0);
        if (latest.hasTargets()) {
          targets.addAll(latest.targets);
        }
      }
    }

    List<Pose2d> poses = new ArrayList<>();
    for (PhotonTrackedTarget target : targets) {
      if (fieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
        Pose2d targetPose = fieldLayout.getTagPose(target.getFiducialId()).get().toPose2d();
        poses.add(targetPose);
      }
    }

    field2d.getObject("tracked targets").setPoses(poses);
  }

  /**
   * Camera Enum to select each camera
   */
  public enum Cameras {
    /**
     * Front Left Camera
     */
    FRONT_LEFT_CAM(
        VisionConstants.FRONT_LEFT_CAMERA_NAME,
        VisionConstants.FRONT_LEFT_CAMERA_ROTATION,
        VisionConstants.FRONT_LEFT_CAMERA_POSITION,
        VisionConstants.SINGLE_TAG_STD_DEVS,
        VisionConstants.MULTI_TAG_STD_DEVS),
    /**
     * Front Right Camera
     */
    FRONT_RIGHT_CAM(
        VisionConstants.FRONT_RIGHT_CAMERA_NAME,
        VisionConstants.FRONT_RIGHT_CAMERA_ROTATION,
        VisionConstants.FRONT_RIGHT_CAMERA_POSITION,
        VisionConstants.SINGLE_TAG_STD_DEVS,
        VisionConstants.MULTI_TAG_STD_DEVS);

    /**
     * Latency alert to use when high latency is detected.
     */
    public final Alert latencyAlert;
    /**
     * Camera instance for comms.
     */
    public final PhotonCamera camera;
    /**
     * Pose estimator for camera.
     */
    public final PhotonPoseEstimator poseEstimator;
    /**
     * Standard Deviation for single tag readings for pose estimation.
     */
    private final Matrix<N3, N1> singleTagStdDevs;
    /**
     * Standard deviation for multi-tag readings for pose estimation.
     */
    private final Matrix<N3, N1> multiTagStdDevs;
    /**
     * Transform of the camera rotation and translation relative to the center of
     * the robot
     */
    private final Transform3d robotToCamTransform;
    /**
     * Current standard deviations used.
     */
    public Matrix<N3, N1> curStdDevs;
    /**
     * Estimated robot pose.
     */
    public Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();

    /**
     * Simulated camera instance which only exists during simulations.
     */
    public PhotonCameraSim cameraSim;
    /**
     * Results list to be updated periodically and cached to avoid unnecessary
     * queries.
     */
    public List<PhotonPipelineResult> resultsList = new ArrayList<>();

    /**
     * Construct a Photon Camera class with help. Standard deviations are fake
     * values, experiment and determine
     * estimation noise on an actual robot.
     *
     * @param name                  Name of the PhotonVision camera found in the PV
     *                              UI.
     * @param robotToCamRotation    {@link Rotation3d} of the camera.
     * @param robotToCamTranslation {@link Translation3d} relative to the center of
     *                              the robot.
     * @param singleTagStdDevs      Single AprilTag standard deviations of estimated
     *                              poses from the camera.
     * @param multiTagStdDevsMatrix Multi AprilTag standard deviations of estimated
     *                              poses from the camera.
     */
    Cameras(String name, Rotation3d robotToCamRotation, Translation3d robotToCamTranslation,
        Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevsMatrix) {
      latencyAlert = new Alert("'" + name + "' Camera is experiencing high latency.", AlertType.kWarning);

      camera = new PhotonCamera(name);

      // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
      robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);

      // PhotonVision 2026 API: Create pose estimator with field layout
      // Strategy is now selected by calling specific estimation methods
      poseEstimator = new PhotonPoseEstimator(
          Vision.fieldLayout,
          robotToCamTransform);

      this.singleTagStdDevs = singleTagStdDevs;
      this.multiTagStdDevs = multiTagStdDevsMatrix;

      if (Robot.isSimulation()) {
        try {
          SimCameraProperties cameraProp = new SimCameraProperties();
          // Arducam OV9281 - 640x480 @ 100 FPS with estimated 90 degree diagonal FOV
          cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(90));
          // Approximate detection noise matching calibration error (avg ~41px from your
          // config)
          cameraProp.setCalibError(0.35, 0.10);
          // Set the camera image capture framerate to match actual cameras
          cameraProp.setFPS(100);
          // The average and standard deviation in milliseconds of image data latency.
          cameraProp.setAvgLatencyMs(35);
          cameraProp.setLatencyStdDevMs(5);

          cameraSim = new PhotonCameraSim(camera, cameraProp);
          cameraSim.enableDrawWireframe(true);
        } catch (Exception e) {
          System.err.println("Warning: PhotonCameraSim initialization failed for " + name + ": " + e.getMessage());
          cameraSim = null;
        }
      }
    }

    /**
     * Add camera to {@link VisionSystemSim} for simulated photon vision.
     *
     * @param systemSim {@link VisionSystemSim} to use.
     */
    public void addToVisionSim(VisionSystemSim systemSim) {
      if (Robot.isSimulation() && cameraSim != null) {
        try {
          systemSim.addCamera(cameraSim, robotToCamTransform);
        } catch (Exception e) {
          System.err.println("Warning: Failed to add camera to VisionSystemSim: " + e.getMessage());
        }
      }
    }

    /**
     * Get the result with the least ambiguity from the best tracked target within
     * the Cache. This may not be the most
     * recent result!
     *
     * @return The result in the cache with the least ambiguous best tracked target.
     *         This is not the most recent result!
     */
    public Optional<PhotonPipelineResult> getBestResult() {
      if (resultsList.isEmpty()) {
        return Optional.empty();
      }

      PhotonPipelineResult bestResult = resultsList.get(0);
      double amiguity = bestResult.getBestTarget().getPoseAmbiguity();
      double currentAmbiguity = 0;
      for (PhotonPipelineResult result : resultsList) {
        currentAmbiguity = result.getBestTarget().getPoseAmbiguity();
        if (currentAmbiguity < amiguity && currentAmbiguity > 0) {
          bestResult = result;
          amiguity = currentAmbiguity;
        }
      }
      return Optional.of(bestResult);
    }

    /**
     * Get the latest result from the current cache.
     *
     * @return Empty optional if nothing is found. Latest result if something is
     *         there.
     */
    public Optional<PhotonPipelineResult> getLatestResult() {
      return resultsList.isEmpty() ? Optional.empty() : Optional.of(resultsList.get(0));
    }

    /**
     * Get the estimated robot pose. Updates the current robot pose estimation,
     * standard deviations, and flushes the
     * cache of results.
     *
     * @return Estimated pose.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
      updateUnreadResults();
      return estimatedRobotPose;
    }

    /**
     * Update the latest results from the camera.
     * Always polls to maintain PhotonVision TimeSync communication.
     * Sorts the list by timestamp (newest first).
     */
    private void updateUnreadResults() {
      // Always call getAllUnreadResults() to maintain TimeSync heartbeat with
      // PhotonVision
      try {
        if (Robot.isReal()) {
          resultsList = camera.getAllUnreadResults();
        } else if (cameraSim != null) {
          resultsList = cameraSim.getCamera().getAllUnreadResults();
        } else {
          // Simulation camera failed to initialize, use empty list
          resultsList = new ArrayList<>();
        }

        // Sort by timestamp descending (newest first) so index 0 is the latest result
        resultsList.sort((PhotonPipelineResult a, PhotonPipelineResult b) -> {
          return Double.compare(b.getTimestampSeconds(), a.getTimestampSeconds());
        });

        if (!resultsList.isEmpty()) {
          updateEstimatedGlobalPose();
        }
      } catch (Exception e) {
        // Gracefully handle any PhotonVision communication errors
        System.err.println("[Vision/" + name() + "] Error getting camera results: " + e.getMessage());
        resultsList = new ArrayList<>();
      }
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be
     * empty. This should only be called once
     * per loop.
     *
     * <p>
     * Also includes updates for the standard deviations, which can (optionally) be
     * retrieved with
     * {@link Cameras#updateEstimationStdDevs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate
     *         timestamp, and targets used for
     *         estimation.
     */
    private void updateEstimatedGlobalPose() {
      Optional<EstimatedRobotPose> visionEst = Optional.empty();

      // Only process the LATEST result (index 0) since resultsList is sorted newest-first
      // Processing all buffered results can cause erratic odometry updates in simulation
      if (!resultsList.isEmpty()) {
        var latestResult = resultsList.get(0);

        // Check if we have any targets before attempting pose estimation
        if (latestResult.hasTargets()) {
          try {
            // PhotonVision 2026 API: Use estimateLowestAmbiguityPose() for robotside pose calculation
            // This selects the pose estimate with lowest ambiguity (most reliable)
            // Works with single or multiple tags without needing camera calibration matrices
            visionEst = poseEstimator.estimateLowestAmbiguityPose(latestResult);
          } catch (Exception e) {
            // Handle pose estimation errors gracefully
            System.err.println("[Vision/" + name() + "] Error estimating pose: " + e.getMessage());
            visionEst = Optional.empty();
          }

          updateEstimationStdDevs(visionEst, latestResult.getTargets());
        }
      }

      estimatedRobotPose = visionEst;
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates
     * dynamic standard deviations based
     * on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets       All targets in this camera frame
     */
    private void updateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
      if (estimatedPose.isEmpty()) {
        // No pose input. Default to single-tag std devs
        curStdDevs = singleTagStdDevs;

      } else {
        // Simplified: Accept all AprilTag detections with configured standard
        // deviations
        int numTags = targets.size();

        if (numTags == 0) {
          // No tags visible. Default to single-tag std devs
          curStdDevs = singleTagStdDevs;
        } else if (numTags > 1) {
          // Multiple tags - use multi-tag std devs (more trustworthy)
          curStdDevs = multiTagStdDevs;
        } else {
          // Single tag - use single-tag std devs
          curStdDevs = singleTagStdDevs;
        }

      }
    }

  }

}
