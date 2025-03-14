package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

/**
 * This class stores predefined Pose2d positions for both blue and red alliances. It provides a
 * method to return the closest pose from the appropriate list based on the current robot pose and
 * the alliance reported by the DriverStation.
 */
public class SidePoseMatcher {

  // Hard-coded list of poses for the blue alliance.
  private static final List<Pose2d> bluePoses =
      Arrays.asList(
          new Pose2d(new Translation2d(4.96, 5.3), new Rotation2d(Math.toRadians(-115.15))),
          new Pose2d(new Translation2d(5.3, 5.07), new Rotation2d(Math.toRadians(-118.87))),
          new Pose2d(new Translation2d(3.6, 5.09), new Rotation2d(Math.toRadians(-54.6))),
          new Pose2d(new Translation2d(3.91, 5.2), new Rotation2d(Math.toRadians(-52.53))),
          new Pose2d(new Translation2d(3.13, 3.76), new Rotation2d(Math.toRadians(5.781))),
          new Pose2d(new Translation2d(3.15, 4.15), new Rotation2d(Math.toRadians(7.12))),
          new Pose2d(new Translation2d(4.04, 2.71), new Rotation2d(Math.toRadians(65.98))),
          new Pose2d(new Translation2d(3.7, 2.95), new Rotation2d(Math.toRadians(63.93))),
          new Pose2d(new Translation2d(3.7, 2.95), new Rotation2d(Math.toRadians(63.93))),
          new Pose2d(new Translation2d(5.4, 2.97), new Rotation2d(Math.toRadians(128.33))),
          new Pose2d(new Translation2d(5.09, 2.79), new Rotation2d(Math.toRadians(128.75))),
          new Pose2d(new Translation2d(5.8, 3.9), new Rotation2d(Math.toRadians(-176.29))),
          new Pose2d(new Translation2d(5.83, 4.3), new Rotation2d(Math.toRadians(-174.66))));

  // Hard-coded list of poses for the red alliance.
  private static final List<Pose2d> redPoses =
      Arrays.asList(
          new Pose2d(new Translation2d(11.72, 3.75), new Rotation2d(Math.toRadians(6.58))),
          new Pose2d(new Translation2d(11.75, 4.06), new Rotation2d(Math.toRadians(4.8))),
          new Pose2d(new Translation2d(13.49, 5.35), new Rotation2d(Math.toRadians(-111.1))),
          new Pose2d(new Translation2d(13.86, 5.09), new Rotation2d(Math.toRadians(-115.1))),
          new Pose2d(new Translation2d(12.64, 2.72), new Rotation2d(Math.toRadians(67.24))),
          new Pose2d(new Translation2d(12.34, 2.93), new Rotation2d(Math.toRadians(67.61))),
          new Pose2d(new Translation2d(12.49, 5.21), new Rotation2d(Math.toRadians(-56.30))),
          new Pose2d(new Translation2d(12.15, 5.07), new Rotation2d(Math.toRadians(-54.18))),
          new Pose2d(new Translation2d(13.62, 2.81), new Rotation2d(Math.toRadians(129.28))),
          new Pose2d(new Translation2d(13.98, 3.01), new Rotation2d(Math.toRadians(127.91))),
          new Pose2d(new Translation2d(14.39, 4.34), new Rotation2d(Math.toRadians(-168.82))),
          new Pose2d(new Translation2d(14.4, 3.92), new Rotation2d(Math.toRadians(-172.02))));

  /**
   * Iterates over a list of Pose2d and returns the one closest to the currentPose.
   *
   * @param poses The list of Pose2d objects.
   * @param currentPose The current robot pose.
   * @return The closest Pose2d from the list.
   */
  private static Pose2d findClosestInList(List<Pose2d> poses, Pose2d currentPose) {
    Pose2d closestPose = null;
    double minDistance = Double.POSITIVE_INFINITY;
    for (Pose2d pose : poses) {
      double distance = currentPose.getTranslation().getDistance(pose.getTranslation());
      if (distance < minDistance) {
        minDistance = distance;
        closestPose = pose;
      }
    }
    return closestPose;
  }

  /**
   * Returns the closest pose to the currentPose from the list corresponding to the alliance. The
   * alliance is retrieved automatically using DriverStation.getAlliance().
   *
   * @param currentPose The current robot pose.
   * @return The closest Pose2d from the appropriate alliance list, or null if the list is empty.
   */
  private static double squaredDistance(Pose2d a, Pose2d b) {
    double dx = a.getTranslation().getX() - b.getTranslation().getX();
    double dy = a.getTranslation().getY() - b.getTranslation().getY();
    return dx * dx + dy * dy;
  }

  public static Pose2d getClosestPose(Pose2d currentPose) {
    DriverStation.Alliance alliance = DriverStation.getAlliance().get();
    List<Pose2d> selectedPoses =
        (alliance == DriverStation.Alliance.Blue)
            ? bluePoses
            : (alliance == DriverStation.Alliance.Red ? redPoses : bluePoses);

    return selectedPoses.stream()
        .min(Comparator.comparingDouble(pose -> squaredDistance(currentPose, pose)))
        .orElse(null);
  }

  public static Pose2d hu() {
    return new Pose2d();
  }
}
