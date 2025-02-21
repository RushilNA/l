package frc.robot.subsystems.flywheel;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
// import frc.robot.subsystems.lookuptable.setpoint;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class shooter extends SubsystemBase {
  private TalonFX intake = new TalonFX(17);
  


  private int distance;
  private Timer time3 = new Timer();

  public shooter() {

    intake.setNeutralMode(NeutralModeValue.Brake);

    
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Intake Current", intake.getStatorCurrent().getValueAsDouble());
    
  }

  public boolean check() {
    return distance > 75;
  }

  public double getDistance() {
    return distance;
  }

  public void speed(double speed) {
    intake.set(speed);
    
  }

  public boolean hasCurrentSpike() {
    
    double current = intake.getStatorCurrent().getValueAsDouble();

    double CURRENT_SPIKE_THRESHOLD = 40.0; 

    return current > CURRENT_SPIKE_THRESHOLD;
  }

}
