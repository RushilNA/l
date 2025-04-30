// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.drive.requests.SysIdSwerveTranslation_Torque;
import frc.robot.subsystems.vision.VisionUtil.VisionMeasurement;
import frc.robot.utils.ArrayBuilder;
import frc.robot.utils.SidePoseMatcher;
import java.io.IOException;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class Drive extends SubsystemBase {
  private final DriveIO io;
  private final DriveIOInputsAutoLogged inputs;
  private final ModuleIOInputsAutoLogged[] modules = ArrayBuilder.buildModuleAutoLogged();

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(Constants.SWERVE_MODULE_OFFSETS);
  private SwerveDrivePoseEstimator poseEstimator = null;
  private Trigger estimatorTrigger =
      new Trigger(() -> poseEstimator != null).and(() -> Constants.currentMode == Mode.REPLAY);
  private SwerveModulePosition[] currentPositions = ArrayBuilder.buildSwerveModulePosition();

  private Alert[] driveDisconnectedAlert =
      ArrayBuilder.buildAlert("Disconnected drive motor on module");
  private Alert[] turnDisconnectedAlert =
      ArrayBuilder.buildAlert("Disconnected turn motor on module");
  private Alert[] turnEncoderDisconnectedAlert =
      ArrayBuilder.buildAlert("Disconnected turn encoder on module");

  private Alert gyroDisconnectedAlert = new Alert("Gyro Disconnected", AlertType.kError);

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  /** Swerve request to apply during robot-centric path following */
  private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds();

  /* Swerve request to apply when braking */
  private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();

  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

  // Example TorqueCurrent SysID - Others are avalible.
  private final SysIdSwerveTranslation_Torque m_translationTorqueCharacterization =
      new SysIdSwerveTranslation_Torque();

  /* SysId routine for characterizing torque translation. This is used to find PID gains for Torque Current of the drive motors. */
  private final SysIdRoutine m_sysIdRoutineTorqueTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(5).per(Second), // Use ramp rate of 5 A/s
              Volts.of(10), // Use dynamic step of 10 A
              Seconds.of(5), // Use timeout of 5 seconds
              // Log state with SignalLogger class
              state -> Logger.recordOutput("SysIdTranslation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output ->
                  setControl(
                      m_translationTorqueCharacterization.withTorqueCurrent(
                          output.in(Volts))), // treat volts as amps
              null,
              this));

  /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
  private final SysIdRoutine m_sysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with Logger class
              state -> Logger.recordOutput("SysIdTranslation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

  /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
  private final SysIdRoutine m_sysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(7), // Use dynamic voltage of 7 V
              null, // Use default timeout (10 s)
              // Log state with Logger class
              state -> Logger.recordOutput("SysIdSteer_State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
   * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
   */
  private final SysIdRoutine m_sysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              /* This is in radians per second squared, but SysId only supports "volts per second" */
              Volts.of(Math.PI / 6).per(Second),
              /* This is in radians per second, but SysId only supports "volts" */
              Volts.of(Math.PI),
              null, // Use default timeout (10 s)
              // Log state with Logger class
              state -> Logger.recordOutput("SysIdRotation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                Logger.recordOutput("Rotational_Rate", output.in(Volts));
              },
              null,
              this));

  /* The SysId routine to test */
  private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

  public Drive(DriveIO io) {

    this.io = io;
    inputs = new DriveIOInputsAutoLogged();

    configureAutoBuilder();
  }

  private void configureAutoBuilder() {
    AutoBuilder.configure(
        this::getPose, // Supplier of current robot pose
        this::resetPose, // Consumer for seeding pose against auto
        this::getChassisSpeeds, // Supplier of current robot speeds
        // Consumer of ChassisSpeeds and feedforwards to drive the robot
        (speeds, feedforwards) ->
            io.setControl(
                m_pathApplyRobotSpeeds
                    .withSpeeds(speeds)
                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
        new PPHolonomicDriveController(
            // PID constants for translation
            new PIDConstants(5, 0, 0),
            // PID constants for rotation
            new PIDConstants(5, 0, 0)),
        Constants.PP_CONFIG,
        // Assume the path needs to be flipped for Red vs Blue, this is normally the case
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this // Subsystem for requirements
        );
  }

  // public Command driveToPose(Pose2d pose) {
  //   // Create the constraints to use while pathfinding
  //   PathConstraints constraints = new PathConstraints(10, 4.0, 5, Units.degreesToRadians(720));

  //   // Since AutoBuilder is configured, we can use it to build pathfinding commands
  //   return AutoBuilder.pathfindToPose(
  //       pose,
  //       constraints,
  //       0.0 // Goal end velocity in meters/se // Rotation delay distance in meters. This is how
  // far
  //       // the robot should travel before attempting to rotate.
  //       );
  // }
  PathPlannerPath path = null;

  public Command driveToPosePathfind(String pathName) {

    // if (SidePoseMatcher.getClosestPose1(getPose()).getX() == 5.244) {
    //   pathName = "Side1R";

    // } else if (SidePoseMatcher.getClosestPose1(getPose()).getX() == 3.704) {
    //   pathName = "Side2R";
    // } else if (SidePoseMatcher.getClosestPose1(getPose()).getX() == 3.300) {
    //   pathName = "Side3R";
    // } else if (SidePoseMatcher.getClosestPose1(getPose()).getX() == 4.022) {
    //   pathName = "Side4R";
    // } else if (SidePoseMatcher.getClosestPose1(getPose()).getX() == 4.936) {
    //   pathName = "Side5R";
    // } else if (SidePoseMatcher.getClosestPose1(getPose()).getX() == 5.619) {
    //   pathName = "Side6R";
    // }

    SmartDashboard.putString("weeeee3", pathName);

    try {

      // Attempt to load the path
      path = PathPlannerPath.fromPathFile(pathName);
    } catch (IOException | org.json.simple.parser.ParseException e) {
      // If there's a file or parsing error, handle it here
      e.printStackTrace();
      // Possibly return an empty command, a default path, etc.
      return null; // or return Commands.none();
    }

    // Create constraints for your path
    PathConstraints constraints =
        new PathConstraints(
            4, // Max velocity (m/s)
            2, // Max acceleration (m/s^2)
            Units.degreesToRadians(720), // Max rotational velocity
            Units.degreesToRadians(720) // Max rotational acceleration
            );

    SmartDashboard.putNumber("weeeee311", SidePoseMatcher.getClosestPose1(getPose()).getX());

    // Now that we have the path, pass it into your pathfinding command
    return AutoBuilder.pathfindThenFollowPath(path, constraints);
  }

  public Command drivetopose(Pose2d pose) {
    // PathPlannerPath path = null;
    // try {
    //   // Attempt to load the path
    //   path = PathPlannerPath.fromPathFile(pathName);
    // } catch (IOException | org.json.simple.parser.ParseException e) {
    //   // If there's a file or parsing error, handle it here
    //   e.printStackTrace();
    //   // Possibly return an empty command, a default path, etc.
    //   return null; // or return Commands.none();
    // }

    // Create constraints for your path
    PathConstraints constraints =
        new PathConstraints(
            4, // Max velocity (m/s)
            5, // Max acceleration (m/s^2)
            Units.degreesToRadians(540), // Max rotational velocity
            Units.degreesToRadians(720) // Max rotational acceleration
            );

    // Now that we have the path, pass it into your pathfinding command
    return AutoBuilder.pathfindToPose(pose, constraints);
  }

  /**
   * Returns a command that applies the specified control request to this swerve drivetrain.
   *
   * @param request Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> io.setControl(requestSupplier.get()));
  }

  public void setControl(SwerveRequest request) {
    io.setControl(request);
  }

  public Command brake() {
    return applyRequest(() -> brakeRequest);
  }

  public void chassisspeed(ChassisSpeeds request) {
    io.setControl(m_pathApplyRobotSpeeds.withSpeeds(request));
  }

  public void drive3(Translation2d translation, double rotation, boolean isOpenLoop) {
    ChassisSpeeds desiredChassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(), translation.getY(), rotation, getPose().getRotation());

    chassisspeed(desiredChassisSpeeds);
  }

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction);
  }

  @Override
  public void periodic() {

    // SmartDashboard.putNumber(
    //     "distance",
    //     SidePoseMatcher.getDistanceToTarget(
    //         getPose(), SidePoseMatcher.getClosestRightPose(getPose())));

    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */

    io.updateInputs(inputs);

    gyroDisconnectedAlert.set(!inputs.gyroConnected);

    io.updateModules(modules);
    for (int i = 0; i < modules.length; i++) {
      Logger.processInputs("Module" + i, modules[i]);
      driveDisconnectedAlert[i].set(!modules[i].driveConnected);
      turnDisconnectedAlert[i].set(!modules[i].turnConnected);
      turnEncoderDisconnectedAlert[i].set(!modules[i].turnEncoderConnected);
    }

    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                io.setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
              });
    }
    updateWithTime();
  }

  public void resetPose(Pose2d pose) {
    if (estimatorTrigger.getAsBoolean()) {
      poseEstimator.resetPose(pose);
    }
    io.resetPose(pose);
  }

  // public ChassisSpeeds getAlignmentSpeeds(Pose2d desiredpose){
  //   desiredposealighmentpose = desiredpose;
  // }

  public ChassisSpeeds getAlignmentSpeeds(Pose2d desiredPose) {

    SmartDashboard.putNumber("X pose", desiredPose.getX());
    SmartDashboard.putNumber("Y Pose", desiredPose.getY());
    SmartDashboard.putNumber("Rotation", desiredPose.getRotation().getDegrees());

    // Use the teleop auto align controller from your constants
    // Note: The third parameter here is the desired linear speed (set to 0 if you only want to
    // rotate)
    return Constants.TELEOP_AUTO_ALIGN.TELEOP_AUTO_ALIGN_CONTROLLER.calculate(
        getPose(), desiredPose, 0, desiredPose.getRotation());
  }

  public ChassisSpeeds getAlignmentSpeedsl2(Pose2d desiredPose) {
    // Use the teleop auto align controller from your constants
    // Note: The third parameter here is the desired linear speed (set to 0 if you only want to
    // rotate)
    return Constants.TELEOP_AUTO_ALIGNl2.TELEOP_AUTO_ALIGN_CONTROLLER.calculate(
        getPose(), desiredPose, 0, desiredPose.getRotation());
  }

  public ChassisSpeeds getAlignmentSpeedsl3(Pose2d desiredPose) {
    // Use the teleop auto align controller from your constants
    // Note: The third parameter here is the desired linear speed (set to 0 if you only want to
    // rotate)
    return Constants.TELEOP_AUTO_ALIGNl3.TELEOP_AUTO_ALIGN_CONTROLLER.calculate(
        getPose(), desiredPose, 0, desiredPose.getRotation());
  }

  // public Command autoAlignToPose(Pose2d targetPose) {
  //   return run(() -> {
  //     // Calculate alignment speeds
  //     ChassisSpeeds speeds = getAlignmentSpeeds(targetPose);
  //     // Optionally limit speeds if needed:
  //     double maxLinear = Constants.OBSERVED_DRIVE_SPEED.in(MetersPerSecond);
  //     speeds.vxMetersPerSecond = MathUtil.clamp(speeds.vxMetersPerSecond, -maxLinear, maxLinear);
  //     speeds.vyMetersPerSecond = MathUtil.clamp(speeds.vyMetersPerSecond, -maxLinear, maxLinear);
  //     // Apply the computed speeds using your IO interface
  //     io.setControl(m_pathApplyRobotSpeeds.withSpeeds(speeds));
  //   })};

  public void stop() {
    io.setControl(brakeRequest);
  }

  public boolean isAtTarget(Pose2d target, Pose2d currentPose) {
    double toleranceMeters = 0.08; // 10 cm tolerance
    return currentPose.getTranslation().getDistance(target.getTranslation()) < toleranceMeters;
  }

  public Command autoAlighnTopose(Pose2d Targetpose) {
    return run(
        () -> {
          ChassisSpeeds speeds = getAlignmentSpeeds(Targetpose);
          double maxLinear = Constants.OBSERVED_DRIVE_SPEED.in(MetersPerSecond);
          speeds.vxMetersPerSecond =
              MathUtil.clamp(speeds.vxMetersPerSecond * 1, -maxLinear, maxLinear);
          speeds.vyMetersPerSecond =
              MathUtil.clamp(speeds.vyMetersPerSecond * 1, -maxLinear, maxLinear);
          io.setControl(m_pathApplyRobotSpeeds.withSpeeds(speeds));
        });
  }

  public Command autoAlighnToposel2(Pose2d Targetpose) {
    return run(
        () -> {
          ChassisSpeeds speeds = getAlignmentSpeedsl2(Targetpose);
          double maxLinear = Constants.OBSERVED_DRIVE_SPEEDl2.in(MetersPerSecond);
          speeds.vxMetersPerSecond =
              MathUtil.clamp(speeds.vxMetersPerSecond * 1, -maxLinear, maxLinear);
          speeds.vyMetersPerSecond =
              MathUtil.clamp(speeds.vyMetersPerSecond * 1, -maxLinear, maxLinear);
          io.setControl(m_pathApplyRobotSpeeds.withSpeeds(speeds));
        });
  }

  public Command autoAlighnToposel3(Pose2d Targetpose) {
    return run(
        () -> {
          ChassisSpeeds speeds = getAlignmentSpeedsl3(Targetpose);
          double maxLinear = Constants.OBSERVED_DRIVE_SPEEDl3.in(MetersPerSecond);
          speeds.vxMetersPerSecond =
              MathUtil.clamp(speeds.vxMetersPerSecond * 1, -maxLinear, maxLinear);
          speeds.vyMetersPerSecond =
              MathUtil.clamp(speeds.vyMetersPerSecond * 1, -maxLinear, maxLinear);
          io.setControl(m_pathApplyRobotSpeeds.withSpeeds(speeds));
        });
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    if (estimatorTrigger.getAsBoolean()) {
      return poseEstimator.getEstimatedPosition();
    }
    return inputs.pose;
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public AngularVelocity getGyroRate() {
    return inputs.gyroRate;
  }

  public Rotation2d getOperatorForwardDirection() {
    return inputs.operatorForwardDirection;
  }

  public Angle[] getDrivePositions() {
    Angle[] values = new Angle[Constants.PP_CONFIG.numModules];
    for (int i = 0; i < values.length; i++) {
      values[i] = modules[i].drivePosition;
    }
    return values;
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  public SwerveModuleState[] getModuleStates() {
    return inputs.moduleStates;
  }

  /** Returns the module target states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Setpoints")
  public SwerveModuleState[] getModuleTarget() {
    return inputs.moduleTargets;
  }

  public void resetgyro() {
    Pose2d pose1 = new Pose2d(getPose().getX(), getPose().getY(), new Rotation2d(0));
    io.resetPose(pose1);
  }

  public void setgyro() {
    Pose2d pose1 = new Pose2d(2, 20, new Rotation2d(30));
    io.resetPose(pose1);
  }

  public SwerveModulePosition[] getModulePositions() {
    return inputs.modulePositions;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return inputs.speeds;
  }

  /**
   * Return the pose at a given timestamp. If the buffer is empty return current pose.
   *
   * @param timestampSeconds The pose's timestamp. This must use WPILib timestamp.
   * @return The pose at the given timestamp (or current pose if the buffer is empty).
   */
  public Pose2d samplePoseAt(double timestampSeconds) {
    return estimatorTrigger.getAsBoolean()
        ? poseEstimator.sampleAt(timestampSeconds).orElse(getPose())
        : io.samplePoseAt(timestampSeconds).orElse(getPose());
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionRobotPoseMeters The measured robot pose from vision
   * @param timestampSeconds The timestamp of the measurement
   * @param visionMeasurementStdDevs Standard deviation matrix for the measurement
   */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    if (estimatorTrigger.getAsBoolean()) {
      poseEstimator.addVisionMeasurement(
          visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    } else {
      io.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(VisionMeasurement visionMeasurement) {
    this.addVisionMeasurement(
        visionMeasurement.poseEstimate().pose().toPose2d(),
        visionMeasurement.poseEstimate().timestampSeconds(),
        visionMeasurement.visionMeasurementStdDevs());
  }

  public void addVisionData(List<VisionMeasurement> visionData) {

    visionData.forEach(this::addVisionMeasurement);
  }

  public VisionParameters getVisionParameters() {
    return new VisionParameters(getPose(), getGyroRate());
  }

  public record VisionParameters(Pose2d robotPose, AngularVelocity gyroRate) {}

  public void updateWithTime() {
    if (Constants.currentMode != Mode.REPLAY || !inputs.odometryIsValid) {
      return;
    }

    if (!estimatorTrigger.getAsBoolean()) {
      poseEstimator =
          new SwerveDrivePoseEstimator(
              kinematics, inputs.pose.getRotation(), inputs.modulePositions, inputs.pose);
    }

    for (int timeIndex = 0; timeIndex < inputs.timestamp.length; timeIndex++) {
      updateModulePositions(timeIndex);
      poseEstimator.updateWithTime(
          inputs.timestamp[timeIndex], inputs.gyroYaw[timeIndex], currentPositions);
    }
  }

  private void updateModulePositions(int timeIndex) {
    for (int moduleIndex = 0; moduleIndex < currentPositions.length; moduleIndex++) {
      currentPositions[moduleIndex].distanceMeters = inputs.drivePositions[moduleIndex][timeIndex];
      currentPositions[moduleIndex].angle = inputs.steerPositions[moduleIndex][timeIndex];
    }
  }
}
