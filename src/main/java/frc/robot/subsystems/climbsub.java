package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
// import frc.robot.subsystems.lookuptable.setpoint;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class climbsub extends SubsystemBase {

  final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  private TalonFX climbm = new TalonFX(16);
  TalonFXConfiguration cfg = new TalonFXConfiguration();

  private Slot0Configs slot0 = cfg.Slot0;

  MotionMagicConfigs motionMagicConfigs = cfg.MotionMagic;
  private CANcoder climbe = new CANcoder(30);

  // idk rn #dont use this id for this motor

  private int distance;
  private Timer time3 = new Timer();

  public climbsub() {
    cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    cfg.Feedback.FeedbackRemoteSensorID = climbe.getDeviceID();

    slot0.kG = 0.2; // A gear ratio of 4:1 results in 0.25 output
    slot0.kS = 0.25;
    slot0.kV = 0.1; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 7; // A position error of 2.5 rotations results in 12 V output
    slot0.kI = 0.1; // no output for integrated error
    slot0.kD = 0; // A velocity error of 1 rps results in 0.1 V output

    motionMagicConfigs.MotionMagicCruiseVelocity = 100; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration =
        300; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 900; // Target jerk of 1600 rps/s/s (0.1 seconds)\

    climbm.getConfigurator().apply(cfg);

    climbm.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("clibm", climbm.getPosition().getValueAsDouble());
  }

  public boolean check() {
    return distance > 75;
  }

  public double getDistance() {
    return distance;
  }

  public void speed(double speed) {
    climbm.set(speed);
  }

  public double velocity() {
    return climbm.getVelocity().getValueAsDouble();
  }

  public Command cmd(double position) {
    return new Command() {
      @Override
      public void initialize() {}

      @Override
      public void execute() {
        climbm.setControl(m_request.withPosition(position).withEnableFOC(true));
      }

      @Override
      public void end(boolean interrupted) {}

      @Override
      public boolean isFinished() {
        return false; // Check if the setpoint is reached
      }
    };
  }

  public boolean hasVelocity(double inputVelo) {

    double Velo = climbm.getVelocity().getValueAsDouble();

    double velocitythreshold = inputVelo;

    return Math.abs(Velo) > velocitythreshold;
  }

  public boolean hasVelocityautoalighn() {

    double Velo = climbm.getVelocity().getValueAsDouble();

    double velocitythreshold = 20;

    return Math.abs(Velo) > velocitythreshold;
  }

  public Command autoncmdOut(double speed, double Velocity) {
    return new Command() {
      @Override
      public void initialize() {}

      @Override
      public void execute() {
        climbm.set(speed);
      }

      @Override
      public void end(boolean interrupted) {
        climbm.set(0);
      }

      @Override
      public boolean isFinished() {
        return hasVelocity(Velocity);
      }
    };
  }

  public Command autoncmdIn(double speed) {
    return new Command() {
      @Override
      public void initialize() {
        time3.reset();
        time3.start();
      }

      @Override
      public void execute() {
        climbm.set(speed);
      }

      @Override
      public void end(boolean interrupted) {
        climbm.set(0);
      }

      @Override
      public boolean isFinished() {
        return time3.get() > 1.25 && Math.abs(climbm.getVelocity().getValueAsDouble()) < 24;
      }
    };
  }
}
