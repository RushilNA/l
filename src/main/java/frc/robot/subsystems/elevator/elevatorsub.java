package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import java.util.List;

public class elevatorsub extends SubsystemBase {
  private List<Double> setpoints1 = List.of(0.0, .0, 0.0, 9.84423828125, 25.44423828125, 5.0);
  private List<Double> setpoints2 = List.of(0.0, 7.75048828125, 7.75048828125, 35.0, 4.5, 5.5);
  private List<Double> activeSetpoints = setpoints1; // Default to setpoints
  private TalonFX le = new TalonFX(14, "Drivetrain");
  private TalonFX re = new TalonFX(13, "Drivetrain");
  private TalonFX flippydoo = new TalonFX(17);
  private PIDController pidup = new PIDController(0.05, 0, 0);
  private PIDController pidauto = new PIDController(0.03, 0, 0);
  private Timer match_time = new Timer();

  TalonFXConfiguration cfg = new TalonFXConfiguration();
  TalonFXConfiguration cfd = new TalonFXConfiguration();

  TalonFXConfiguration cff = new TalonFXConfiguration();
  private final VoltageOut m_sysIdControl = new VoltageOut(0);
  private Slot0Configs slot0 = cfg.Slot0;
  private Slot1Configs slotd = cfd.Slot1;

  private Slot0Configs slot2 = cff.Slot0;
  MotionMagicConfigs motionMagicConfigs = cfg.MotionMagic;
  MotionMagicConfigs motionMagicConfigs21 = cfd.MotionMagic;

  MotionMagicConfigs motionMagicConfigs2 = cff.MotionMagic;
  final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
  final MotionMagicVoltage m_request1 = new MotionMagicVoltage(0);
  private final ProfiledPIDController profiledPID =
      new ProfiledPIDController(0.1, 0.0, 0.0, new TrapezoidProfile.Constraints(8, 7));

  private enum ElevatorState {
    MOVING, // using Motion Magic to drive to a setpoint
    HOLDING // using a WPILib PID controller to hold the position
  }

  private ElevatorState m_state = ElevatorState.HOLDING;

  // The target position (in sensor units) that you want to reach
  private double m_targetPosition = 0.0;

  // Tolerances for switching from MOVING to HOLDING
  private final double kPositionTolerance = 5; // adjust as needed
  private final double kVelocityTolerance = 0.1;

  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(0.01), // Reduce dynamic voltage to 4 to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with Phoenix SignalLogger class
              state -> SignalLogger.writeString("Elevator State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> le.setControl(m_sysIdControl.withOutput(volts)), null, this));
  private final SysIdRoutine m_sysIdRoutine2 =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(1), // Reduce dynamic voltage to 4 to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with Phoenix SignalLogger class
              state -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> re.setControl(m_sysIdControl.withOutput(volts)), null, this));

  public elevatorsub() {
    slot0.kG = 0.5; // A gear ratio of 4:1 results in 0.25 output
    slot0.kS = 3;
    slot0.kV = 1.8; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0.04; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 8; // A position error of 2.5 rotations results in 12 V output
    slot0.kI = 0; // no output for integrated error
    slot0.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    slot2.kG = 0.1; // A gear ratio of 4:1 results in 0.25 output
    slot2.kS = 0.25;
    slot2.kV = 0.18; // A velocity target of 1 rps results in 0.12 V output
    slot2.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot2.kP = 1.8; // A position error of 2.5 rotations results in 12 V output
    slot2.kI = 0.1; // no output for integrated error
    slot2.kD = 0; // A velocity error of 1 rps results in 0.1 V output

    // cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.Fus
    motionMagicConfigs2.MotionMagicCruiseVelocity = 100;
    motionMagicConfigs2.MotionMagicAcceleration = 300;
    motionMagicConfigs2.MotionMagicJerk = 900;

    motionMagicConfigs.MotionMagicCruiseVelocity = 500; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration =
        400; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 900; // Target jerk of 1600 rps/s/s (0.1 seconds)\

    // cfg.Feedback.FeedbackRemoteSensorID = hi.getDeviceID();

    le.getConfigurator().apply(cfg);
    re.getConfigurator().apply(cfg);
    flippydoo.getConfigurator().apply(cff);
    flippydoo.setNeutralMode(NeutralModeValue.Brake);
    le.setNeutralMode(NeutralModeValue.Brake);
    re.setNeutralMode(NeutralModeValue.Brake);

    // Follower followrequest = new Follower(le.getDeviceID(), true);

    match_time.start();
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("leftvoltage", le.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("rightvoltage", re.getMotorVoltage().getValueAsDouble());

    SmartDashboard.putNumber("left", le.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("right", -re.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("voltage1", le.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Pivot", flippydoo.getPosition().getValueAsDouble());
    SmartDashboard.putBoolean("pivoitcheck", flipcheck(-11.679));
    SmartDashboard.putBoolean(
        "Sate Checker", Constants.getRobotState() == Constants.RobotState.ALGEA);

    re.setControl(new Follower(le.getDeviceID(), true));
    SmartDashboard.putNumber("Match timer", match_time.getMatchTime());

    SmartDashboard.putBoolean("check for autonn", autoncheck(1) && flipcheck(-4.13134765625));
  }

  public void resetenc() {
    le.setPosition(0);
    re.setPosition(0);
  }

  public double getLeftPosition() {
    return le.getPosition().getValueAsDouble();
  }

  public void setMotionMagic(double position) {
    // Use Motion Magic with feedforward and FOC enabled.
    le.setControl(m_request.withPosition(position).withFeedForward(0.15).withEnableFOC(true));
    // If you want the follower to track, you can do the same for 're' if needed.
  }

  public void setMotionMagic1(int position) {

    // Use Motion Magic with feedforward and FOC enabled.
    le.setControl(
        m_request
            .withPosition(activeSetpoints.get(position))
            .withFeedForward(0.6)
            .withEnableFOC(true));

    // If you want the follower to track, you can do the same for 're' if needed.
  }

  public void setMotionMagicflip(double position) {
    // Use Motion Magic with feedforward and FOC enabled.
    flippydoo.setControl(m_request.withPosition(position).withEnableFOC(true).withFeedForward(0.4));
    // If you want the follower to track, you can do the same for 're' if needed.
  }

  public void setsetpoint(double position) {
    pidup.setSetpoint(position);
  }

  public void setposition(double position) {
    pidup.setSetpoint(position);
  }

  public void setsetpointauto(double position) {
    profiledPID.setGoal(position);
  }

  public void transposepid() {
    double speed1 = profiledPID.calculate(flippydoo.getPosition().getValueAsDouble());
    flippydoo.set(speed1);
  }

  public void pid() {
    double speed1 = pidup.calculate(flippydoo.getPosition().getValueAsDouble());
    flippydoo.set(speed1);
  }

  public void pid21() {
    double speed1 = pidup.calculate(le.getPosition().getValueAsDouble());
    le.set(speed1);
  }

  public void pid2() {
    double speed1 = pidauto.calculate(flippydoo.getPosition().getValueAsDouble());
    flippydoo.set(speed1);
  }

  public boolean flipcheck(double position) {
    double curentpos = flippydoo.getPosition().getValueAsDouble();
    if (curentpos - 0.8 < position && curentpos + 0.8 > position) {
      return true;

    } else {
      return false;
    }
  }

  public boolean autoalighncheck(int targeposition) {
    return targeposition(targeposition) + 0.3 < Math.abs(le.getPosition().getValueAsDouble())
        || targeposition(targeposition) - 0.3 < le.getPosition().getValueAsDouble();
  }

  public void initializePid(int position) {
    pidup.reset();
    pidup.setSetpoint(targeposition(position));
  }

  public void setPidOutput(int position) {
    pidup.setSetpoint(targeposition(position));
    double speed = pidup.calculate(le.getPosition().getValueAsDouble());
    le.set(speed);
  }

  public void stop() {
    le.setControl(new VoltageOut(0));
    re.setControl(new VoltageOut(0));
  }

  public int whichlist() {
    if (activeSetpoints == setpoints1) {
      return 1;
    } else {
      return 2;
    }
  }

  public Command cmd(double position) {
    return new Command() {
      @Override
      public void initialize() {
        // Initialization code, such as resetting encoders or PID controllers
        // int kErrThreshold = 10; // how many sensor units until its close-enough
        // int kLoopsToSettle = 2; // how many loops sensor must be close-enough
        // int _withinThresholdLoops = 0;
      }

      @Override
      public void execute() {
        // check(position);
        re.setPosition(-re.getPosition().getValueAsDouble());

        le.setControl(m_request.withPosition(position).withFeedForward(0.15));
        re.setControl(m_request.withPosition(position).withFeedForward(0.15));
      }

      @Override
      public void end(boolean interrupted) {}

      @Override
      public boolean isFinished() {
        return false; // Check if the setpoint is reached
      }
    };
  }

  public Command cmdf(double position) {
    return new Command() {
      @Override
      public void initialize() {
        // Initialization code, such as resetting encoders or PID controllers
        // int kErrThreshold = 10; // how many sensor units until its close-enough
        // int kLoopsToSettle = 2; // how many loops sensor must be close-enough
        // int _withinThresholdLoops = 0;
        pidup.setSetpoint(position);
      }

      @Override
      public void execute() {

        double speed = pidup.calculate(flippydoo.getPosition().getValueAsDouble());
        flippydoo.set(speed);
        // check(position);

      }

      @Override
      public void end(boolean interrupted) {
        flippydoo.set(0);
      }

      @Override
      public boolean isFinished() {
        return false; // Check if the setpoint is reached
      }
    };
  }

  public Command Flipydo(double targetPosition) {
    return new Command() {
      // Define a tolerance (adjust as needed based on your sensor units)
      private final double kTolerance = 0.1;

      @Override
      public void initialize() {
        // Optionally reset any state or encoders if needed

      }

      @Override
      public void execute() {
        // Command the leader motor using Motion Magic with feedforward.
        // (Since re is meant to follow le, remove direct control of re here.)

        flippydoo.setControl(m_request.withPosition(targetPosition).withEnableFOC(true));
      }

      @Override
      public boolean isFinished() {
        // End the command once the error is within tolerance.
        double error = Math.abs(le.getPosition().getValueAsDouble() - targetPosition);
        return false;
      }

      @Override
      public void end(boolean interrupted) {
        // Once finished (or interrupted), stop the motors.
        le.setControl(new VoltageOut(0));
        // If you’re using a follower, you can let it follow automatically.
        // Alternatively, if you’re controlling 're' elsewhere, ensure it’s set to zero.
        re.setControl(new VoltageOut(0));
      }
    };
  }

  public Command CTREpid(double targetPosition) {
    return new Command() {
      // Define a tolerance (adjust as needed based on your sensor units)
      private final double kTolerance = 0.1;

      @Override
      public void initialize() {
        // Optionally reset any state or encoders if needed

      }

      @Override
      public void execute() {
        // Command the leader motor using Motion Magic with feedforward.
        // (Since re is meant to follow le, remove direct control of re here.)

        le.setControl(m_request.withPosition(targetPosition).withEnableFOC(true));
      }

      @Override
      public boolean isFinished() {
        // End the command once the error is within tolerance.
        double error = Math.abs(le.getPosition().getValueAsDouble() - targetPosition);
        return false;
      }

      @Override
      public void end(boolean interrupted) {
        // Once finished (or interrupted), stop the motors.
        le.setControl(new VoltageOut(0));
        // If you’re using a follower, you can let it follow automatically.
        // Alternatively, if you’re controlling 're' elsewhere, ensure it’s set to zero.
        re.setControl(new VoltageOut(0));
      }
    };
  }

  public void apply() {
    le.getConfigurator().apply(cfg);
    re.getConfigurator().apply(cfg);
  }

  public double targeposition(int index) {
    return activeSetpoints.get(index);
  }

  public boolean autoncheck(int index) {
    double sensor = le.getPosition().getValueAsDouble();
    return targeposition(index) - 0.3 < sensor;
  }

  public boolean autoncheck2(double index) {
    double sensor = le.getPosition().getValueAsDouble();
    return index * 0.75 - 0.3 < sensor;
  }
  // 5
  // 4.7
  // 4.8

  public boolean autoncheckposition(int index) {
    double sensor = le.getPosition().getValueAsDouble();
    return targeposition(index) - 0.3 < sensor;
  }

  public boolean elecheck(int index) {
    double sensor = le.getPosition().getValueAsDouble();
    return sensor + 0.1 < targeposition(index);
  }

  public Command Motionmagictoggle(int value) {
    return new Command() {

      // Define a tolerance (adjust as needed based on your sensor units)
      private final double kTolerance = 0.25;

      @Override
      public void initialize() {

        // Optionally reset any state or encoders if needed

      }

      @Override
      public void execute() {

        // Command the leader motor using Motion Magic with feedforward.
        // (Since re is meant to follow le, remove direct control of re here.)

        le.setControl(
            m_request
                .withPosition(activeSetpoints.get(value))
                .withFeedForward(0.4)
                .withEnableFOC(true));
      }

      @Override
      public boolean isFinished() {
        // End the command once the error is within tolerance.
        double error = Math.abs(le.getPosition().getValueAsDouble() - activeSetpoints.get(value));
        return kTolerance > error;
      }

      @Override
      public void end(boolean interrupted) {
        // Once finished (or interrupted), stop the motors.
        le.setControl(new VoltageOut(0));
        // If you’re using a follower, you can let it follow automatically.
        // Alternatively, if you’re controlling 're' elsewhere, ensure it’s set to zero.
        re.setControl(new VoltageOut(0));
      }
    };
  }

  public Command Motionmagic(double targetPosition) {
    return new Command() {
      // Define a tolerance (adjust as needed based on your sensor units)
      private final double kTolerance = 0.25;

      @Override
      public void initialize() {
        // Optionally reset any state or encoders if needed
        pidup.setSetpoint(targetPosition);
      }

      @Override
      public void execute() {

        // Command the leader motor using Motion Magic with feedforward.
        // (Since re is meant to follow le, remove direct control of re here.)

        le.setControl(
            m_request.withPosition(targetPosition).withFeedForward(0.4).withEnableFOC(true));
      }

      @Override
      public boolean isFinished() {
        // End the command once the error is within tolerance.
        double error = Math.abs(le.getPosition().getValueAsDouble() - targetPosition);
        return false;
      }
    };
  }

  public Command Motionmagic2(double targetPosition) {
    return new Command() {
      // Define a tolerance (adjust as needed based on your sensor units)
      private final double kTolerance = 0.25;

      @Override
      public void initialize() {
        // Optionally reset any state or encoders if needed
        pidup.setSetpoint(targetPosition);
      }

      @Override
      public void execute() {

        // Command the leader motor using Motion Magic with feedforward.
        // (Since re is meant to follow le, remove direct control of re here.)

        flippydoo.setControl(
            m_request.withPosition(targetPosition).withFeedForward(0.4).withEnableFOC(true));
      }

      @Override
      public boolean isFinished() {
        // End the command once the error is within tolerance.
        double position = le.getPosition().getValueAsDouble();
        return flipcheck(targetPosition);
      }

      @Override
      public void end(boolean interrupted) {
        // Once finished (or interrupted), stop the motors.

      }
    };
  }

  public Command pidup(double targetPosition) {
    return new Command() {
      // Define a tolerance (adjust as needed based on your sensor units)
      private final double kTolerance = 0.1;

      @Override
      public void initialize() {
        // Optionally reset any state or encoders if needed
        pidup.setSetpoint(targetPosition);
      }

      @Override
      public void execute() {
        // Command the leader motor using Motion Magic with feedforward.
        // (Since re is meant to follow le, remove direct control of re here.)

        double speed = pidup.calculate(le.getPosition().getValueAsDouble());
        le.set(speed);
      }

      @Override
      public boolean isFinished() {
        // End the command once the error is within tolerance.
        double error = Math.abs(le.getPosition().getValueAsDouble() - targetPosition);
        return false;
      }

      @Override
      public void end(boolean interrupted) {
        // Once finished (or interrupted), stop the motors.
        le.setControl(new VoltageOut(0));
        // If you’re using a follower, you can let it follow automatically.
        // Alternatively, if you’re controlling 're' elsewhere, ensure it’s set to zero.
        re.setControl(new VoltageOut(0));
      }
    };
  }

  public Command speed(double targetPosition) {
    return new Command() {
      // Define a tolerance (adjust as needed based on your sensor units)
      private final double kTolerance = 0.25;

      @Override
      public void initialize() {
        // Optionally reset any state or encoders if needed
        pidup.setSetpoint(targetPosition);
      }

      @Override
      public void execute() {

        // Command the leader motor using Motion Magic with feedforward.
        // (Since re is meant to follow le, remove direct control of re here.)

        le.set(targetPosition);
      }

      @Override
      public boolean isFinished() {
        // End the command once the error is within tolerance.

        return false;
      }

      @Override
      public void end(boolean interrupted) {
        // Once finished (or interrupted), stop the motors.
        le.set(0);
        // If you’re using a follower, you can let it follow automatically.
        // Alternatively, if you’re controlling 're' elsewhere, ensure it’s set to zero.

      }
    };
  }

  public Command runCurrentZeroing() {
    return this.run(
            () -> {
              // Run: Command the leader to drive downward at -1.0 V.
              le.setControl(m_request.withPosition(0));
            })
        .until(
            () -> {
              // Continue running until the current exceeds 40 A.
              // Adjust the method call based on your Phoenix 6 API (e.g., getStatorCurrent() or
              // getSupplyCurrent()).
              return le.getStatorCurrent().getValueAsDouble() > 40;
            })
        .andThen(
            new InstantCommand(
                () -> {
                  le.setControl(new VoltageOut(0));
                  re.setControl(new VoltageOut(0));
                }))
        // Wait for 0.5 seconds.
        .andThen(new WaitCommand(0.5))
        // After waiting, reset the encoders.
        .andThen(
            new InstantCommand(
                () -> {
                  le.setPosition(0.0);
                  re.setPosition(0.0);
                }))
        .beforeStarting(
            () -> {
              // Before starting, perform any one-time actions. For example, set a locking servo:
              // io.setLockServoRotation(0.2);
              // (Replace with your actual method if you have a servo to engage.)
              System.out.println("Starting currxent zeroing routine");
            });
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public void togglesetpoint() {
    if (activeSetpoints == setpoints1) {
      activeSetpoints = setpoints2;

      Constants.setRobotState(Constants.RobotState.ALGEA);
    } else {
      activeSetpoints = setpoints1;
      Constants.setRobotState(Constants.RobotState.IDLE);
    }
  }

  public Boolean checkifsetpoint1() {
    return activeSetpoints == setpoints1;
  }

  public Boolean check(double position) {
    double curentpos = le.getPosition().getValueAsDouble();
    if (curentpos - 0.2 < position && curentpos + 0.2 > position) {
      return true;

    } else {
      return false;
    }
  }
}
