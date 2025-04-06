/* (C) Robolancers 2025 */
package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.commands.ReefAlign;
import frc.robot.util.VirtualSubsystem;
import java.util.function.Consumer;
import java.util.function.Supplier;

@Logged
public class Vision extends VirtualSubsystem {
  private final VisionIO io;

  private final Consumer<VisionEstimate> visionDataConsumer;
  private final Consumer<VisionEstimate> reefVisionDataConsumer;

  public static Vision create(
      Supplier<Pose2d> robotPoseSupplier,
      Consumer<VisionEstimate> visionDataConsumer,
      Consumer<VisionEstimate> reefVisionDataConsumer) {
    return RobotBase.isReal()
        ? new Vision(
            new VisionIOReal(VisionConstants.kCameraConfigs),
            visionDataConsumer,
            reefVisionDataConsumer)
        : new Vision(
            new VisionIOSim(robotPoseSupplier, VisionConstants.kCameraConfigs),
            visionDataConsumer,
            reefVisionDataConsumer);
  }

  private Vision(
      VisionIO io,
      Consumer<VisionEstimate> visionDataConsumer,
      Consumer<VisionEstimate> reefVisionDataConsumer) {
    this.io = io;
    this.visionDataConsumer = visionDataConsumer;
    this.reefVisionDataConsumer = reefVisionDataConsumer;
  }

  @Override
  public void periodic() {
    final var latestEstimates = io.getLatestEstimates();

    for (final var est : latestEstimates) {
      visionDataConsumer.accept(est);

      if (est.sourceType() == CameraUsage.REEF && isValidReefPose(est)) {
        reefVisionDataConsumer.accept(est);
      }
    }
  }

  public boolean areCamerasConnected() {
    return io.areCamerasConnected();
  }

  private static boolean isValidReefPose(VisionEstimate estimate) {
    if (estimate.estimate().targetsUsed.size() == 0) return false;
    Alliance alliance = DriverStation.getAlliance().orElse(null);
    boolean blueContains =
        ReefAlign.kBlueReefTagIDs.contains(estimate.estimate().targetsUsed.get(0).fiducialId);
    boolean redContains =
        ReefAlign.kRedReefTagIDs.contains(estimate.estimate().targetsUsed.get(0).fiducialId);
    return estimate.estimate().targetsUsed.size() > 1
        || (alliance == null
            ? (blueContains || redContains)
            : alliance == Alliance.Red ? redContains : blueContains);
  }
}
