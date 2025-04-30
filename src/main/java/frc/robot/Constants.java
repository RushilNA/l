// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {

  public final class elevatorconstants {
    public double l1 = 0;
    public double l2 = -26.4;
    public double l3 = -26.4;
    public double l4 = -23.3831054687;
  }

  public static final double l1 = 0;
  public static final double l2 = -26.4;
  public static final double l3 = -26.4;
  public static final double l4 = -23.3831054687;
  public static final LinearVelocity OBSERVED_DRIVE_SPEED = Units.MetersPerSecond.of(1.3);
  public static final LinearVelocity OBSERVED_DRIVE_SPEEDl2 = Units.MetersPerSecond.of(0.2);
  public static final LinearVelocity OBSERVED_DRIVE_SPEEDl3 = Units.MetersPerSecond.of(1);

  public static final AngularVelocity TURN_SPEED = Units.DegreesPerSecond.of(720);

  public static class TELEOP_AUTO_ALIGN {
    // TODO: Test if this actually works LOL
    public static final LinearVelocity DESIRED_AUTO_ALIGN_SPEED =
        Units.MetersPerSecond.of(OBSERVED_DRIVE_SPEED.in(MetersPerSecond) / 4);

    public static final Distance MAX_AUTO_DRIVE_CORAL_STATION_DISTANCE = Units.Meters.of(10);
    public static final Distance MAX_AUTO_DRIVE_REEF_DISTANCE = Units.Meters.of(1);
    public static final Distance MAX_AUTO_DRIVE_PROCESSOR_DISTANCE = Units.Meters.of(5);
    public static final LinearVelocity MIN_DRIVER_OVERRIDE = OBSERVED_DRIVE_SPEED.div(10);

    public static final PIDController TRANS_CONTROLLER = new PIDController(3.5, 0, 0);
    public static final Distance AT_POINT_TOLERANCE = Units.Inches.of(0.01);

    public static final ProfiledPIDController ROTATION_CONTROLLER =
        new ProfiledPIDController(
            4,
            0,
            0,
            new TrapezoidProfile.Constraints(
                TURN_SPEED.in(Units.DegreesPerSecond),
                Math.pow(TURN_SPEED.in(Units.DegreesPerSecond), 2)));
    public static final Angle AT_ROTATION_TOLERANCE = Units.Degrees.of(1);

    public static final Distance AUTO_ALIGNMENT_TOLERANCE = Units.Inches.of(0.5);

    static {
      TRANS_CONTROLLER.setTolerance(AT_POINT_TOLERANCE.in(Units.Meters));

      ROTATION_CONTROLLER.enableContinuousInput(0, 1000);
      ROTATION_CONTROLLER.setTolerance(AT_ROTATION_TOLERANCE.in(Units.Degrees));
    }

    public static HolonomicDriveController TELEOP_AUTO_ALIGN_CONTROLLER =
        new HolonomicDriveController(TRANS_CONTROLLER, TRANS_CONTROLLER, ROTATION_CONTROLLER);
  }

  public static class TELEOP_AUTO_ALIGNl2 {
    // TODO: Test if this actually works LOL
    public static final LinearVelocity DESIRED_AUTO_ALIGN_SPEED =
        Units.MetersPerSecond.of(OBSERVED_DRIVE_SPEEDl2.in(MetersPerSecond) / 4);

    public static final Distance MAX_AUTO_DRIVE_CORAL_STATION_DISTANCE = Units.Meters.of(10);
    public static final Distance MAX_AUTO_DRIVE_REEF_DISTANCE = Units.Meters.of(1);
    public static final Distance MAX_AUTO_DRIVE_PROCESSOR_DISTANCE = Units.Meters.of(5);
    public static final LinearVelocity MIN_DRIVER_OVERRIDE = OBSERVED_DRIVE_SPEED.div(10);

    public static final PIDController TRANS_CONTROLLER = new PIDController(6.5, 0, 0);
    public static final Distance AT_POINT_TOLERANCE = Units.Inches.of(0.01);

    public static final ProfiledPIDController ROTATION_CONTROLLER =
        new ProfiledPIDController(
            4,
            0,
            0,
            new TrapezoidProfile.Constraints(
                TURN_SPEED.in(Units.DegreesPerSecond),
                Math.pow(TURN_SPEED.in(Units.DegreesPerSecond), 2)));
    public static final Angle AT_ROTATION_TOLERANCE = Units.Degrees.of(1);

    public static final Distance AUTO_ALIGNMENT_TOLERANCE = Units.Inches.of(0.5);

    static {
      TRANS_CONTROLLER.setTolerance(AT_POINT_TOLERANCE.in(Units.Meters));

      ROTATION_CONTROLLER.enableContinuousInput(0, 1000);
      ROTATION_CONTROLLER.setTolerance(AT_ROTATION_TOLERANCE.in(Units.Degrees));
    }

    public static HolonomicDriveController TELEOP_AUTO_ALIGN_CONTROLLER =
        new HolonomicDriveController(TRANS_CONTROLLER, TRANS_CONTROLLER, ROTATION_CONTROLLER);
  }

  public static class TELEOP_AUTO_ALIGNl3 {
    // TODO: Test if this actually works LOL
    public static final LinearVelocity DESIRED_AUTO_ALIGN_SPEED =
        Units.MetersPerSecond.of(OBSERVED_DRIVE_SPEEDl3.in(MetersPerSecond) / 4);

    public static final Distance MAX_AUTO_DRIVE_CORAL_STATION_DISTANCE = Units.Meters.of(10);
    public static final Distance MAX_AUTO_DRIVE_REEF_DISTANCE = Units.Meters.of(1);
    public static final Distance MAX_AUTO_DRIVE_PROCESSOR_DISTANCE = Units.Meters.of(5);
    public static final LinearVelocity MIN_DRIVER_OVERRIDE = OBSERVED_DRIVE_SPEED.div(10);

    public static final PIDController TRANS_CONTROLLER = new PIDController(6.5, 0, 0);
    public static final Distance AT_POINT_TOLERANCE = Units.Inches.of(0.01);

    public static final ProfiledPIDController ROTATION_CONTROLLER =
        new ProfiledPIDController(
            4,
            0,
            0,
            new TrapezoidProfile.Constraints(
                TURN_SPEED.in(Units.DegreesPerSecond),
                Math.pow(TURN_SPEED.in(Units.DegreesPerSecond), 2)));
    public static final Angle AT_ROTATION_TOLERANCE = Units.Degrees.of(1);

    public static final Distance AUTO_ALIGNMENT_TOLERANCE = Units.Inches.of(0.5);

    static {
      TRANS_CONTROLLER.setTolerance(AT_POINT_TOLERANCE.in(Units.Meters));

      ROTATION_CONTROLLER.enableContinuousInput(0, 1000);
      ROTATION_CONTROLLER.setTolerance(AT_ROTATION_TOLERANCE.in(Units.Degrees));
    }

    public static HolonomicDriveController TELEOP_AUTO_ALIGN_CONTROLLER =
        new HolonomicDriveController(TRANS_CONTROLLER, TRANS_CONTROLLER, ROTATION_CONTROLLER);
  }

  public static final Mode simMode = Mode.SIM;

  public static final AngularVelocity MaxAngularRate = RotationsPerSecond.of(3);
  public static final AngularVelocity MaxModuleRate = RotationsPerSecond.of(20.0);

  // PathPlanner config constants
  private static final Mass ROBOT_MASS = Kilogram.of(65.771);
  // 15.2
  private static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(6.0);
  private static final double WHEEL_COF = 1.916;
  public static final SwerveModuleConstants SWERVE_MODULE_CONSTANTS = TunerConstants.FrontLeft;
  public static final Translation2d[] SWERVE_MODULE_OFFSETS =
      new Translation2d[] {
        new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
        new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
        new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
        new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
      };

  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));
  public static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS,
          ROBOT_MOI,
          new ModuleConfig(
              SWERVE_MODULE_CONSTANTS.WheelRadius,
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1).withReduction(SWERVE_MODULE_CONSTANTS.DriveMotorGearRatio),
              SWERVE_MODULE_CONSTANTS.SlipCurrent,
              1),
          SWERVE_MODULE_OFFSETS);

  public static final SwerveSetpointGenerator setpointGenerator =
      new SwerveSetpointGenerator(
          Constants.PP_CONFIG, edu.wpi.first.math.util.Units.rotationsToRadians(10.0));

  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static enum RobotState {
    IDLE, // Robot is not doing anything2
    MOVING, // Robot is driving
    INTAKING, // Robot is picking up a game piece
    SHOOTING, // Robot is shooting a game piece
    CLIMBING,
    ALGEA; // Robot is climbing
  }

  public static enum autovision {
    None, // Robot is not doing anything2
    Holding
  }

  public static enum Alagestate {
    None, // Robot is not doing anything2
    Holding
  }

  public static enum coralstate {
    None, // Robot is not doing anything2
    Holding
  }

  public enum Elevatorposition {
    L4,
    L3,
    L2,
    L1, // using Motion Magic to drive to a setpoint
    L0 // using a WPILib PID controller to hold the position
  }

  private static Elevatorposition curentElevatorposition = Elevatorposition.L0;

  private static RobotState currentRobotState = RobotState.IDLE;

  private static Alagestate currentalagestate = Alagestate.None;

  private static coralstate currentcoralstate = coralstate.Holding;
  private static autovision currentvision = autovision.Holding;

  public static Elevatorposition getElevatorState() {
    return curentElevatorposition;
  }

  public static autovision getvisionstate() {
    return currentvision;
  }

  public static Alagestate getAlgaestate() {
    return currentalagestate;
  }

  public static coralstate getCoralstate() {
    return currentcoralstate;
  }

  public static void setElevatorState(Elevatorposition newState) {
    curentElevatorposition = newState;
    System.out.println("Robot state updated to: " + newState);
  }

  public static void setvisionsate(autovision newState) {

    currentvision = newState;
    System.out.println("auto state updated to: " + newState);
  }

  public static final class Pose {
    public static final int pigeonID = 1;

    public static final PIDController rotationPID =
        new PIDController(0.003, 0.0, 0.0); // kI was 0.050 for NCCMP 2024
    public static final double rotationKS = 0.02;
    public static final double rotationIZone = 2.0; // degrees

    public static final double tiltWarning = 5.0;
    public static final double tiltError = 10.0;

    public static final double fieldWidth =
        edu.wpi.first.math.util.Units.inchesToMeters(26 * 12 + 5);
    public static final double fieldLength =
        edu.wpi.first.math.util.Units.inchesToMeters(57 * 12 + 6.875);

    public static enum ReefFace {
      AB,
      CD,
      EF,
      GH,
      IJ,
      KL
    }

    public static final double reefElevatorZoneRadius =
        edu.wpi.first.math.util.Units.inchesToMeters(65.0);
    public static final double autoUpDistance = edu.wpi.first.math.util.Units.inchesToMeters(24.0);
    public static final double wingLength = edu.wpi.first.math.util.Units.inchesToMeters(280);

    // Locations from the Blue Alliance perspective
    public static final Translation2d reefCenter =
        new Translation2d(edu.wpi.first.math.util.Units.inchesToMeters(176.75), fieldWidth / 2.0);
  }

  /** Returns the current robot state. */
  public static RobotState getRobotState() {
    return currentRobotState;
  }

  /** Sets the current robot state. */
  public static void setRobotState(RobotState newState) {
    currentRobotState = newState;
    System.out.println("Robot state updated to: " + newState);
  }

  public static void setAlgaestate(Alagestate newState) {
    currentalagestate = newState;
  }

  public static void setCoralstate(coralstate newState) {
    currentcoralstate = newState;
  }

  static {
    // Checks to make sure config matches GUI values. Code should not throw as not breaking
    if (!PP_CONFIG.hasValidConfig()) {
      String error = "Invalid robot configuration detected in PP_CONFIG";
      System.err.println(error);
    }
  }
}
