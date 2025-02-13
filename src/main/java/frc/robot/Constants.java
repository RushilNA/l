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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
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

  public static final Mode simMode = Mode.SIM;

  public static final AngularVelocity MaxAngularRate = RotationsPerSecond.of(1.5);
  public static final AngularVelocity MaxModuleRate = RotationsPerSecond.of(20.0);

  // PathPlanner config constants
  private static final Mass ROBOT_MASS = Kilogram.of(69.78);
  // 15.2
  private static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(6.0);
  private static final double WHEEL_COF = 1.9;
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
      new SwerveSetpointGenerator(Constants.PP_CONFIG, Units.rotationsToRadians(10.0));

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

  public enum Elevatorposition {
    Troph, // using Motion Magic to drive to a setpoint
    Anythingelse // using a WPILib PID controller to hold the position
  }

  private static Elevatorposition curentElevatorposition = Elevatorposition.Anythingelse;

  private static RobotState currentRobotState = RobotState.IDLE;

  public static Elevatorposition getElevatorState() {
    return curentElevatorposition;
  }

  public static void setElevatorState(Elevatorposition newState) {
    curentElevatorposition = newState;
    System.out.println("Robot state updated to: " + newState);
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

  static {
    // Checks to make sure config matches GUI values. Code should not throw as not breaking
    if (!PP_CONFIG.hasValidConfig()) {
      String error = "Invalid robot configuration detected in PP_CONFIG";
      System.err.println(error);
    }
  }
}
