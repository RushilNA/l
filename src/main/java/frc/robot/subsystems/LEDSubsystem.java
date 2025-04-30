package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotState;

public class LEDSubsystem extends SubsystemBase {
  private final CANdle candle = new CANdle(0, "Drivetrain");

  // Variables for flash behavior during state transitions (between ALGEA and IDLE)
  private RobotState lastRobotState = Constants.getRobotState();
  private boolean flashMode = false;
  private final Timer flashTimer = new Timer();
  private final double flashDuration = 2.0; // seconds for flash mode

  // Blink timer for fast flash effect during the transition
  private final Timer blinkTimer = new Timer();
  private boolean blinkOff = false;
  private final double blinkInterval = 0.1; // seconds

  // Animations for different robot states:
  // - Disabled uses an RGBFade animation.
  // - IDLE uses a Rainbow animation.
  // - ALGEA uses a Fire animation.
  private final Animation rgbFadeAnimation = new RgbFadeAnimation(1.5, 0.5, 850);
  private final Animation rainbowAnimation = new RainbowAnimation(1.5, 1, 850, false, 8);
  private final Animation fireAnimation = new FireAnimation(1.5, 4, 850, 0.8, 0.2, false, 8);

  // A simple Color class for setting a solid color during flash mode.
  public static class Color {
    public final int R, G, B;

    public Color(int r, int g, int b) {
      R = r;
      G = g;
      B = b;
    }
  }

  public enum CandleSelection {
    LEFT,
    RIGHT
  }

  // Predefined colors for flash mode.
  // When transitioning, the LED will alternate between off and a flash color.
  public static final class Colors {
    public static final Color off = new Color(0, 0, 0);
    // Flash color for ALGEA mode (matching a fire-like hue)
    public static final Color fireFlash = new Color(255, 64, 0);
    // Flash color for IDLE mode (bright white)
    public static final Color rainbowFlash = new Color(255, 255, 255);
  }

  public LEDSubsystem() {
    // Use device 0 for LEFT and device 1 for RIGHT.
    applyConfigs();
    blinkTimer.start();

    // Set initial animation based on the current state.
    RobotState initialState = Constants.getRobotState();
    if (!DriverStation.isEnabled()) {
      animate(fireAnimation);
    } else {
      if (initialState == RobotState.ALGEA) {
        animate(rgbFadeAnimation);
      } else if (initialState == RobotState.IDLE) {
        animate(fireAnimation);
      } else {
        animate(fireAnimation);
      }
    }
  }

  private void applyConfigs() {
    CANdleConfiguration config = new CANdleConfiguration();
    config.brightnessScalar = 1.0;
    config.stripType = LEDStripType.GRB;
    config.v5Enabled = true;
    config.disableWhenLOS = false; // Adjust if needed.
    candle.configAllSettings(config);
  }

  public void animate(Animation animation) {
    candle.animate(animation);
  }

  public void clearAnimation() {
    candle.clearAnimation(0);
  }

  // Sets the LED to a specific solid color (used during flash mode or by commands).
  public void setColor(Color color) {
    candle.setLEDs(color.R, color.G, color.B);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("LED/RobotState", Constants.getRobotState().toString());

    // If the robot is disabled, always run the RGBFade animation.
    if (!DriverStation.isEnabled()) {
      flashMode = false; // Cancel any flash mode if active.
      animate(fireAnimation);
    } else {
      RobotState currentState = Constants.getRobotState();

      if (currentState == Constants.RobotState.ALGEA) {
        animate(rainbowAnimation);
      }
      if (currentState == Constants.RobotState.IDLE) {
        animate(fireAnimation);
      }
    }
  }

  /**
   * Returns a command that sets the LED color to yellow.
   *
   * <p>When this command is scheduled, the LEDs will immediately change to yellow (RGB: 255, 255,
   * 0).
   */
  public Command getSetYellowCommand() {
    return new InstantCommand(() -> setColor(new Color(255, 255, 0)));
  }

  // Alternatively, you can define a nested command class if you prefer:
  public class SetYellowCommand extends InstantCommand {
    public SetYellowCommand() {
      super(() -> setColor(new Color(255, 255, 0)), LEDSubsystem.this);
    }
  }
}
