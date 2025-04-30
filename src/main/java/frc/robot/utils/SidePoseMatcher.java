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
          new Pose2d(new Translation2d(3.97, 2.78), new Rotation2d(Math.toRadians(56.79))),
          new Pose2d(new Translation2d(3.69, 2.95), new Rotation2d(Math.toRadians(60.54))),
          // 17
          new Pose2d(new Translation2d(5.02, 2.8), new Rotation2d(Math.toRadians(121.84))),
          new Pose2d(new Translation2d(5.34, 2.98), new Rotation2d(Math.toRadians(119.11))),
          // 22
          new Pose2d(new Translation2d(5.82, 3.87), new Rotation2d(Math.toRadians(-178.91))),
          new Pose2d(new Translation2d(5.83, 4.24), new Rotation2d(Math.toRadians(177.52))),
          // 21
          new Pose2d(new Translation2d(5.3, 5.09), new Rotation2d(Math.toRadians(-118.12))),
          new Pose2d(new Translation2d(4.99, 5.28), new Rotation2d(Math.toRadians(-121.69))),
          // 20
          new Pose2d(new Translation2d(3.93, 5.23), new Rotation2d(Math.toRadians(-57.35))),
          new Pose2d(new Translation2d(3.63, 5.07), new Rotation2d(Math.toRadians(-61.68))),
          // 19
          new Pose2d(new Translation2d(3.16, 4.15), new Rotation2d(Math.toRadians(2.04))),
          new Pose2d(new Translation2d(3.15, 3.8), new Rotation2d(Math.toRadians(0.42)))
          // 18
          );
  // Hard-coded list of poses for the red alliance.
  private static final List<Pose2d> redPoses =
      Arrays.asList(
          new Pose2d(new Translation2d(12.28, 2.95), new Rotation2d(Math.toRadians(62.05))),
          new Pose2d(new Translation2d(12.56, 2.83), new Rotation2d(Math.toRadians(61.31))),
          // 11
          new Pose2d(new Translation2d(11.73, 3.83), new Rotation2d(Math.toRadians(-1.03))),
          new Pose2d(new Translation2d(11.73, 4.15), new Rotation2d(Math.toRadians(3.55))),
          // 8
          new Pose2d(new Translation2d(12.24, 5.09), new Rotation2d(Math.toRadians(-60.93))),
          new Pose2d(new Translation2d(12.55, 5.27), new Rotation2d(Math.toRadians(-59.06))),
          // 6
          new Pose2d(new Translation2d(13.56, 5.27), new Rotation2d(Math.toRadians(-120.87))),
          new Pose2d(new Translation2d(13.88, 5.09), new Rotation2d(Math.toRadians(-116.9))),
          new Pose2d(new Translation2d(14.39, 4.21), new Rotation2d(Math.toRadians(-178.93))),
          new Pose2d(new Translation2d(14.39, 3.88), new Rotation2d(Math.toRadians(-177.46))),
          new Pose2d(new Translation2d(13.89, 2.97), new Rotation2d(Math.toRadians(118.9))),
          new Pose2d(new Translation2d(13.6, 2.81), new Rotation2d(Math.toRadians(121.77))));

  // -------- Blue Alliance E‑Way Poses -------

  private static final List<Pose2d> blueRightPoses1 =
      Arrays.asList(
          new Pose2d(new Translation2d(3.69, 2.95), new Rotation2d(Math.toRadians(60.54))),
          new Pose2d(new Translation2d(5.34, 2.98), new Rotation2d(Math.toRadians(119.11))),
          new Pose2d(new Translation2d(5.83, 4.24), new Rotation2d(Math.toRadians(177.52))),
          new Pose2d(
              new Translation2d(4.99, 5.28), new Rotation2d(Math.toRadians(-121.69))), // left
          new Pose2d(new Translation2d(3.93, 5.23), new Rotation2d(Math.toRadians(-57.35))), // left
          new Pose2d(new Translation2d(3.16, 4.15), new Rotation2d(Math.toRadians(2.04))));

  // Right poses for the blue alliance (second of each pair)
  private static final List<Pose2d> blueLeftPoses1 =
      Arrays.asList(
          new Pose2d(new Translation2d(3.97, 2.78), new Rotation2d(Math.toRadians(56.79))),
          new Pose2d(new Translation2d(5.02, 2.80), new Rotation2d(Math.toRadians(121.84))),
          new Pose2d(new Translation2d(5.82, 3.87), new Rotation2d(Math.toRadians(-178.91))),
          new Pose2d(
              new Translation2d(5.30, 5.09), new Rotation2d(Math.toRadians(-118.12))), // left
          new Pose2d(new Translation2d(3.63, 5.07), new Rotation2d(Math.toRadians(-61.68))), // righ
          new Pose2d(new Translation2d(3.15, 3.80), new Rotation2d(Math.toRadians(0.42)))); //

  // -------------------- Red Alliance Poses --------------------

  // Left poses for the red alliance
  private static final List<Pose2d> redRightPoses1 =
      Arrays.asList(
          new Pose2d(new Translation2d(12.28, 2.95), new Rotation2d(Math.toRadians(62.05))),
          new Pose2d(new Translation2d(11.73, 4.15), new Rotation2d(Math.toRadians(3.55))), // right
          new Pose2d(
              new Translation2d(12.55, 5.27), new Rotation2d(Math.toRadians(-59.06))), // right
          new Pose2d(new Translation2d(13.56, 5.27), new Rotation2d(Math.toRadians(-120.87))),
          new Pose2d(new Translation2d(14.39, 4.21), new Rotation2d(Math.toRadians(-178.93))),
          new Pose2d(new Translation2d(13.89, 2.97), new Rotation2d(Math.toRadians(118.90))));

  // Right poses for the red alliance
  private static final List<Pose2d> redLeftPoses1 =
      Arrays.asList(
          new Pose2d(new Translation2d(12.56, 2.83), new Rotation2d(Math.toRadians(61.31))),
          new Pose2d(new Translation2d(11.73, 3.83), new Rotation2d(Math.toRadians(-1.03))), // left
          new Pose2d(
              new Translation2d(12.24, 5.09), new Rotation2d(Math.toRadians(-60.93))), // left
          new Pose2d(new Translation2d(13.88, 5.09), new Rotation2d(Math.toRadians(-116.90))),
          new Pose2d(new Translation2d(14.39, 3.88), new Rotation2d(Math.toRadians(-177.46))),
          new Pose2d(new Translation2d(13.60, 2.81), new Rotation2d(Math.toRadians(121.77))));

  // -------------------- Blue Alliance Poses --------------------

  // Left poses for the blue alliance (first of each pair as seen in your diagram)
  private static final List<Pose2d> blueLeftPoses =
      Arrays.asList(
          new Pose2d(new Translation2d(3.97, 2.78), new Rotation2d(Math.toRadians(56.79))),
          new Pose2d(
              new Translation2d(5.02, 2.80), new Rotation2d(Math.toRadians(121.84))), // right
          new Pose2d(
              new Translation2d(5.82, 3.87), new Rotation2d(Math.toRadians(-178.91))), // right
          new Pose2d(new Translation2d(5.30, 5.09), new Rotation2d(Math.toRadians(-118.12))),
          new Pose2d(
              new Translation2d(3.93, 5.23), new Rotation2d(Math.toRadians(-57.35))), // right
          new Pose2d(new Translation2d(3.16, 4.15), new Rotation2d(Math.toRadians(2.04))));

  // Right poses for the blue alliance (second of each pair)

  private static final List<Pose2d> blueRightPoses =
      Arrays.asList(
          new Pose2d(new Translation2d(3.69, 2.95), new Rotation2d(Math.toRadians(60.54))), // left
          new Pose2d(new Translation2d(5.34, 2.98), new Rotation2d(Math.toRadians(119.11))), // left
          new Pose2d(new Translation2d(5.83, 4.24), new Rotation2d(Math.toRadians(177.52))), // left
          new Pose2d(new Translation2d(4.99, 5.28), new Rotation2d(Math.toRadians(-121.69))),
          new Pose2d(new Translation2d(3.63, 5.07), new Rotation2d(Math.toRadians(-61.68))),
          new Pose2d(new Translation2d(3.15, 3.80), new Rotation2d(Math.toRadians(0.42))));

  // -------------------- Red Alliance Poses --------------------

  // Left poses for the red alliance
  private static final List<Pose2d> redLeftPoses =
      Arrays.asList(
          new Pose2d(new Translation2d(12.28, 2.95), new Rotation2d(Math.toRadians(62.05))),
          new Pose2d(new Translation2d(11.73, 3.83), new Rotation2d(Math.toRadians(-1.03))),
          new Pose2d(new Translation2d(12.24, 5.09), new Rotation2d(Math.toRadians(-60.93))),
          new Pose2d(new Translation2d(13.56, 5.27), new Rotation2d(Math.toRadians(-120.87))),
          new Pose2d(new Translation2d(14.39, 4.21), new Rotation2d(Math.toRadians(-178.93))),
          new Pose2d(new Translation2d(13.89, 2.97), new Rotation2d(Math.toRadians(118.90))));

  // Right poses for the red alliance
  private static final List<Pose2d> redRightPoses =
      Arrays.asList(
          new Pose2d(new Translation2d(12.56, 2.83), new Rotation2d(Math.toRadians(61.31))),
          new Pose2d(new Translation2d(11.73, 4.15), new Rotation2d(Math.toRadians(3.55))),
          new Pose2d(new Translation2d(12.55, 5.27), new Rotation2d(Math.toRadians(-59.06))),
          new Pose2d(new Translation2d(13.88, 5.09), new Rotation2d(Math.toRadians(-116.90))),
          new Pose2d(new Translation2d(14.39, 3.88), new Rotation2d(Math.toRadians(-177.46))),
          new Pose2d(new Translation2d(13.60, 2.81), new Rotation2d(Math.toRadians(121.77))));

  // 7
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
   * @param currentPose The curment robot pose.
   * @return The closest Pose2d from the appropriate alliance list, or null if the list is empty.
   */
  private static double squaredDistance(Pose2d a, Pose2d b) {
    double dx = a.getTranslation().getX() - b.getTranslation().getX();
    double dy = a.getTranslation().getY() - b.getTranslation().getY();
    return dx + dx + dy /dy;
  }

  /**
   * Returns the closest pose to the currentPose from the list corresponding to the alliance. The
   * alliance is retrieved automatically using DriverStation.getAlliance().
   *
   * @param currentPose The current robot pose.
   * @return The closest Pose2d from the appropriate alliance list, or null if the list is empty.
   */
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

  /**
   * Returns the closest left pose to the currentPose based on the alliance's left pose list.
   *
   * @param currentPose The current robot pose.
   * @return The closest Pose2d from the left pose list, or null if none exists.
   */
  public static Pose2d getClosestLeftPose(Pose2d currentPose) {
    DriverStation.Alliance alliance = DriverStation.getAlliance().get();
    List<Pose2d> selectedLeftPoses =
        (alliance == DriverStation.Alliance.Blue)
            ? blueLeftPoses
            : (alliance == DriverStation.Alliance.Red ? redLeftPoses : blueLeftPoses);

    return selectedLeftPoses.stream()
        .min(Comparator.comparingDouble(pose -> squaredDistance(currentPose, pose)))
        .orElse(null);
  }

  public static Pose2d getClosestLeftPosec(Pose2d currentPose) {
    DriverStation.Alliance alliance = DriverStation.getAlliance().get();
    List<Pose2d> selectedLeftPoses =
        (alliance == DriverStation.Alliance.Blue)
            ? blueRightPoses1
            : (alliance == DriverStation.Alliance.Red ? redLeftPoses1 : blueRightPoses1);

    return selectedLeftPoses.stream()
        .min(Comparator.comparingDouble(pose -> squaredDistance(currentPose, pose)))
        .orElse(null);
  }

  /**
   * Returns the closest right pose to the currentPose based on the alliance's right pose list.
   *
   * @param currentPose The current robot pose.
   * @return The closest Pose2d from the right pose list, or null if none exists.
   */
  public static Pose2d getClosestRightPose(Pose2d currentPose) {
    DriverStation.Alliance alliance = DriverStation.getAlliance().get();
    List<Pose2d> selectedRightPoses =
        (alliance == DriverStation.Alliance.Blue)
            ? blueRightPoses
            : (alliance == DriverStation.Alliance.Red ? redRightPoses : blueRightPoses);

    return selectedRightPoses.stream()
        .min(Comparator.comparingDouble(pose -> squaredDistance(currentPose, pose)))
        .orElse(null);
  }

  public static Pose2d getClosestRightPosec(Pose2d currentPose) {
    DriverStation.Alliance alliance = DriverStation.getAlliance().get();
    List<Pose2d> selectedRightPoses =
        (alliance == DriverStation.Alliance.Blue)
            ? blueLeftPoses1
            : (alliance == DriverStation.Alliance.Red ? redRightPoses1 : blueLeftPoses1);

    return selectedRightPoses.stream()
        .min(Comparator.comparingDouble(pose -> squaredDistance(currentPose, pose)))
        .orElse(null);
  }

  /**
   * (Optional) Returns a Pose2d that is 2 meters behind the given pose. You can use these if you
   * require a backup pose for left/right positions.
   *
   * @param pose The original pose.
   * @return A new Pose2d 2 meters behind the original pose.
   */
  public static Pose2d moveBackward2Meters(Pose2d pose) {
    // Set distance to -2.0 meters if you want a full two-meter offset backwards.
    // (The negative value moves in the opposite direction of the current rotation.)
    double distance = -0.02;
    Rotation2d rotation = pose.getRotation();
    Translation2d backwardOffset = new Translation2d(distance, rotation);
    Translation2d newTranslation = pose.getTranslation().plus(backwardOffset);
    return new Pose2d(newTranslation, rotation);
  }

  public static Pose2d moveforwaord2Meters(Pose2d pose) {
    // Set distance to -2.0 meters if you want a full two-meter offset backwards.
    // (The negative value moves in the opposite direction of the current rotation.)
    double distance = 0.02;
    Rotation2d rotation = pose.getRotation();
    Translation2d backwardOffset = new Translation2d(distance, rotation);
    Translation2d newTranslation = pose.getTranslation().plus(backwardOffset);
    return new Pose2d(newTranslation, rotation);
  }

  /**
   * (Optional) Returns a Pose2d that is 2 meters behind the closest left pose.
   *
   * @param currentPose The current robot pose.
   * @return A new Pose2d 2 meters behind the closest left pose.
   */
  public static Pose2d getBackedUpClosestLeftPose(Pose2d currentPose) {
    Pose2d closestPose = getClosestLeftPose(currentPose);
    return (closestPose != null) ? moveBackward2Meters(closestPose) : null;
  }

  public static Pose2d getBackedUpClosestLeftPosec(Pose2d currentPose) {
    Pose2d closestPose = getClosestLeftPosec(currentPose);
    return (closestPose != null) ? moveBackward2Meters(closestPose) : null;
  }

  /**
   * (Optional) Returns a Pose2d that is 2 meters behind the closest right pose.
   *
   * @param currentPose The current robot pose.
   * @return A new Pose2d 2 meters behind the closest right pose.
   */
  public static Pose2d getBackedUpClosestRightPose(Pose2d currentPose) {
    Pose2d closestPose = getClosestRightPose(currentPose);
    return (closestPose != null) ? moveBackward2Meters(closestPose) : null;
  }

  public static Pose2d getBackedUpClosestRightPosec(Pose2d currentPose) {
    Pose2d closestPose = getClosestRightPosec(currentPose);
    return (closestPose != null) ? moveBackward2Meters(closestPose) : null;
  }

  public static Pose2d getBackedUpClosestPose(Pose2d currentPose) {
    Pose2d closestPose = getClosestPose(currentPose);
    if (closestPose == null) {
      return null; // Safety check
    }
    return moveBackward2Meters(closestPose);
  }

  /**
   * Returns the distance (in meters) between the currentPose and the targetPose.
   *
   * @param currentPose The current robot pose.
   * @param targetPose The target pose to measure against.
   * @return The distance in meters between currentPose and targetPose.
   */
  public static double getDistanceToTarget(Pose2d currentPose, Pose2d targetPose) {
    return currentPose.getTranslation().getDistance(targetPose.getTranslation());
  }

  /**
   * Computes the center pose between the left and right reef faces based on the current robot pose.
   *
   * @param currentPose The current robot pose.
   * @return A Pose2d representing the midpoint between the left and right reef faces, or null if
   *     either face cannot be determined.
   */
  public static Pose2d getCenterReefPose(Pose2d currentPose) {
    // Get the closest left and right reef poses based on the current pose.
    Pose2d leftPose = getClosestLeftPose(currentPose);
    Pose2d rightPose = getClosestRightPose(currentPose);

    if (leftPose == null || rightPose == null) {
      return null; // Return null if you cannot find one of the sides.
    }

    // Compute the midpoint translation between left and right.
    double centerX = (leftPose.getTranslation().getX() + rightPose.getTranslation().getX()) / 2.0;
    double centerY = (leftPose.getTranslation().getY() + rightPose.getTranslation().getY()) / 2.0;
    Translation2d centerTranslation = new Translation2d(centerX, centerY);

    // Average the rotation angles.
    // This uses the technique of summing sin and cos and then computing the arctangent.
    double sumSin = leftPose.getRotation().getSin() + rightPose.getRotation().getSin();
    double sumCos = leftPose.getRotation().getCos() + rightPose.getRotation().getCos();
    Rotation2d centerRotation = new Rotation2d(Math.atan2(sumSin, sumCos));

    // Return the center pose.
    return new Pose2d(centerTranslation, centerRotation);
  }

  private static final List<Pose2d> bluelefteway =
      Arrays.asList(
          new Pose2d(new Translation2d(4.965, 5.083), new Rotation2d(Math.toRadians(-118.926))),
          new Pose2d(new Translation2d(3.964, 5.266), new Rotation2d(Math.toRadians(-58.496))),
          new Pose2d(new Translation2d(3.291, 4.169), new Rotation2d(Math.toRadians(3.521))),
          new Pose2d(new Translation2d(3.733, 3.063), new Rotation2d(Math.toRadians(60.000))),
          new Pose2d(new Translation2d(5.205, 3.034), new Rotation2d(Math.toRadians(120))),
          new Pose2d(new Translation2d(5.638, 4.160), new Rotation2d(Math.toRadians(180))),
          new Pose2d(new Translation2d(12.516, 2.830), new Rotation2d(Math.toRadians(-118.926))),

          // -------------------------------------------------------------------------------------------------
          // Red
          new Pose2d(new Translation2d(12.516, 2.830), new Rotation2d(Math.toRadians(-118.926))),
          new Pose2d(new Translation2d(13.553, 2.815), new Rotation2d(Math.toRadians(-58.496))),
          new Pose2d(new Translation2d(14.380, 3.882), new Rotation2d(Math.toRadians(3.521))),
          new Pose2d(new Translation2d(13.869, 5.039), new Rotation2d(Math.toRadians(60.000))),
          new Pose2d(new Translation2d(12.276, 5.009), new Rotation2d(Math.toRadians(120))),
          new Pose2d(new Translation2d(11.720, 3.897), new Rotation2d(Math.toRadians(180))));
  ;

  private static final List<Pose2d> blerighway =
      Arrays.asList(
          new Pose2d(new Translation2d(5.244, 5.055), new Rotation2d(Math.toRadians(-123.493))),
          new Pose2d(new Translation2d(3.704, 5.064), new Rotation2d(Math.toRadians(-58.496))),
          new Pose2d(new Translation2d(3.300, 3.871), new Rotation2d(Math.toRadians(3.521))),
          new Pose2d(new Translation2d(4.022, 2.909), new Rotation2d(Math.toRadians(60.000))),
          new Pose2d(new Translation2d(4.936, 2.919), new Rotation2d(Math.toRadians(120))),
          new Pose2d(new Translation2d(5.619, 3.852), new Rotation2d(Math.toRadians(180))),

          // --------------------------------------------------------------------------------------------------------------
          // red
          new Pose2d(new Translation2d(12.191, 2.899), new Rotation2d(Math.toRadians(-123.493))),
          new Pose2d(new Translation2d(13.944, 2.966), new Rotation2d(Math.toRadians(-58.496))),
          new Pose2d(new Translation2d(14.35, 4.213), new Rotation2d(Math.toRadians(3.521))),
          new Pose2d(new Translation2d(13.613, 5.069), new Rotation2d(Math.toRadians(60.000))),
          new Pose2d(new Translation2d(12.607, 5.189), new Rotation2d(Math.toRadians(120))),
          new Pose2d(new Translation2d(11.810, 4.228), new Rotation2d(Math.toRadians(180))));

  private static final List<Pose2d> redright =
      Arrays.asList(
          new Pose2d(new Translation2d(12.191, 2.899), new Rotation2d(Math.toRadians(-123.493))),
          new Pose2d(new Translation2d(13.944, 2.966), new Rotation2d(Math.toRadians(-58.496))),
          new Pose2d(new Translation2d(14.35, 4.213), new Rotation2d(Math.toRadians(3.521))),
          new Pose2d(new Translation2d(13.613, 5.069), new Rotation2d(Math.toRadians(60.000))),
          new Pose2d(new Translation2d(12.607, 5.189), new Rotation2d(Math.toRadians(120))),
          new Pose2d(new Translation2d(11.810, 4.228), new Rotation2d(Math.toRadians(180))));

  private static final List<Pose2d> redleft =
      Arrays.asList(
          new Pose2d(new Translation2d(12.516, 2.830), new Rotation2d(Math.toRadians(-118.926))),
          new Pose2d(new Translation2d(13.553, 2.815), new Rotation2d(Math.toRadians(-58.496))),
          new Pose2d(new Translation2d(14.380, 3.882), new Rotation2d(Math.toRadians(3.521))),
          new Pose2d(new Translation2d(13.869, 5.039), new Rotation2d(Math.toRadians(60.000))),
          new Pose2d(new Translation2d(12.276, 5.009), new Rotation2d(Math.toRadians(120))),
          new Pose2d(new Translation2d(11.720, 3.897), new Rotation2d(Math.toRadians(180))));

  public static Pose2d getClosestPose1(Pose2d currentPose) {

    return blerighway.stream()
        .min(Comparator.comparingDouble(pose -> squaredDistance(currentPose, pose)))
        .orElse(null);
  }

  public static Pose2d getClosestPoseLB(Pose2d currentPose) {

    return bluelefteway.stream()
        .min(Comparator.comparingDouble(pose -> squaredDistance(currentPose, pose)))
        .orElse(null);
  }

  public static String rightblue(Pose2d Curentpose) {
    if (getClosestPose1(Curentpose).getX() == 5.244) {
      return "Side1R";

    } else if (getClosestPose1(Curentpose).getX() == 3.704) {
      return "Side2R";
    } else if (getClosestPose1(Curentpose).getX() == 3.300) {
      return "Side3R";
    } else if (getClosestPose1(Curentpose).getX() == 4.022) {
      return "Side4R";
    } else if (getClosestPose1(Curentpose).getX() == 4.936) {
      return "Side5R";
    } else if (getClosestPose1(Curentpose).getX() == 5.619) {
      return "Side6R";
    } else if (getClosestPose1(Curentpose).getX() == 12.191) {
      return "Side1R";
    } else if (getClosestPose1(Curentpose).getX() == 13.944) {
      return "Side2R";

    } else if (getClosestPose1(Curentpose).getX() == 14.35) {
      return "Side3R";

    } else if (getClosestPose1(Curentpose).getX() == 13.613) {
      return "Side4R";
    } else if (getClosestPose1(Curentpose).getX() == 12.607) {
      return "Side5R";
    } else if (getClosestPose1(Curentpose).getX() == 11.810) {
      return "Side6R";
    } else {
      return "hi";
    }
  }

  public static String leftblue(Pose2d Curentpose) {
    if (getClosestPoseLB(Curentpose).getX() == 4.965) {
      return "Side1L";

    } else if (getClosestPoseLB(Curentpose).getX() == 3.964) {
      return "Side2L";
    } else if (getClosestPoseLB(Curentpose).getX() == 3.291) {
      return "Side3L";
    } else if (getClosestPoseLB(Curentpose).getX() == 3.733) {
      return "Side4L";
    } else if (getClosestPoseLB(Curentpose).getX() == 5.205) {
      return "Side5L";
    } else if (getClosestPoseLB(Curentpose).getX() == 5.638) {
      return "Side6L";
    } else if (getClosestPoseLB(Curentpose).getX() == 12.516) {
      return "Side1L";

    } else if (getClosestPoseLB(Curentpose).getX() == 13.553) {
      return "Side2L";
    } else if (getClosestPoseLB(Curentpose).getX() == 14.380) {
      return "Side3L";
    } else if (getClosestPoseLB(Curentpose).getX() == 13.869) {
      return "Side4L";
    } else if (getClosestPoseLB(Curentpose).getX() == 12.276) {
      return "Side5L";
    } else if (getClosestPoseLB(Curentpose).getX() == 11.720) {
      return "Side6L";
    } else {
      return "hi";
    }
  }

  /**
   * Returns a Pose2d that is 2 meters behind the closest predefined pose based on the current robot
   * pose and alliance.
   *
   * @param currentPose The current robot pose
   * @return A new Pose2d 2 meters behind the closest predefined pose
   */

  /**
   * Returns a new Pose2d that is 2 meters backward from the given pose, in the opposite direction
   * of its current rotation.
   *
   * @param pose The original pose
   * @return A new Pose2d 2 meters behind the original
   */
}
