package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.elevatorsub;

public class Elevatorcmd extends Command {
  private final elevatorsub elevator;

  private int targetposition;
  private final double tolerance = 0.25; // Tolerance to switch from Motion Magic to PID
  private double l0 = 0;

  private double l1 = -7.13134765625;
  private double l2 = -26.8;
  private double l3 = -26.4;
  private double l4 = -23.58310546875;

  private boolean first;
  private boolean up;

  private double flipsetpoint;

  private enum State {
    MOVING,
    HOLDING
  }

  private State currentState;

  /**
   * Creates a new ElevatorMoveAndHoldCommand.
   *
   * @param elevator The elevator subsystem.
   * @param targetPosition The target position (in sensor units) to move to.
   */
  public Elevatorcmd(elevatorsub elevator, boolean hi) {
    this.up = hi;
    this.elevator = elevator;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {

    targetposition = elevator.elevatorpos();

    if (elevator.whichlist() == 1) {

      Constants.setRobotState(Constants.RobotState.IDLE);
    } else if (elevator.whichlist() == 2) {
      Constants.setRobotState(Constants.RobotState.ALGEA);
    }

    if (elevator.elevatorpos() == 1) {
      Constants.setElevatorState(Constants.Elevatorposition.L1);
    } else if (elevator.elevatorpos() == 2) {
      Constants.setElevatorState(Constants.Elevatorposition.L2);
    } else if (elevator.elevatorpos() == 3) {
      Constants.setElevatorState(Constants.Elevatorposition.L3);
    } else if (elevator.elevatorpos() == 4) {
      Constants.setElevatorState(Constants.Elevatorposition.L4);

      // Normal
    } else {
      Constants.setElevatorState(Constants.Elevatorposition.L0);
    }
    // Start in the MOVING state and reset encoders if needed.
    currentState = State.MOVING;
    if (elevator.getLeftPosition() < 0.1) {
      elevator.resetenc();
    }

    first = true;
  }

  @Override
  public void execute() {

    if (up) {
      // Set flipsetpoint based on the desired elevator state.
      if (elevator.elevatorpos() == 1) {
        flipsetpoint = l1;

      } else if (elevator.elevatorpos() == 2) {
        flipsetpoint = l2;
      } else if (elevator.elevatorpos() == 3) {
        flipsetpoint = l3;
      } else if (elevator.elevatorpos() == 4) {
        flipsetpoint = l4;

        // BargeShoot

      }

      // Check if the flip motor has reached its setpoint.
      // Note: Use flipsetpoint (not targetPosition) for the check.
      if (!elevator.flipcheck(flipsetpoint)) {

        // Command the flip motor until it is at its setpoint.
        elevator.setMotionMagicflip(flipsetpoint);
        // Do not start moving the elevator until the flip motor is ready.
        return;
      }

      // Once the flip motor is holding its setpoint, command the elevator.

      elevator.setMotionMagic1(elevator.elevatorpos());
      elevator.setMotionMagicflip(flipsetpoint);
      // When close enough to the target, switch to PID holding mode.
      // if (Math.abs(currentPos - targetPosition) < tolerance) {
      //   currentState = State.HOLDING;
      //   elevator.initializePid(targetPosition);
      // } else if (currentState == State.HOLDING) {
      //   // Use PID to hold the position.
      //   elevator.Motionmagictoggle(targetPosition);
    } else if (up == false) {
      elevator.Motionmagic(0);
      if (elevator.check(0)) {
        elevator.setMotionMagicflip(-0.4);
        // idk
      }
    }
  }

  @Override
  public boolean isFinished() {
    // This command runs until it is interrupted (for example, by another command).
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the elevator when the command ends.

  }
}
