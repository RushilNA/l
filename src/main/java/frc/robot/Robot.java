// Robot.java
package frc.robot;

import au.grapplerobotics.CanBridge;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutonElevatorcmd;
import frc.robot.commands.AutonomusElevatorcmd;
import frc.robot.commands.Elevatorcmd;
import frc.robot.commands.Hyper;
import frc.robot.commands.Hyperl3;
import frc.robot.commands.Lynkalighnmentleft;
import frc.robot.commands.barge;
import frc.robot.commands.l3algae;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.ScourceCam;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOCTRE;
import frc.robot.subsystems.arm.ArmIOSIM;
import frc.robot.subsystems.arm.algee;
import frc.robot.subsystems.climbsub;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSIM;
import frc.robot.subsystems.elevator.elevatorsub;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSIM;
import frc.robot.subsystems.flywheel.shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.utils.LocalADStarAK;
import frc.robot.utils.SidePoseMatcher;
import frc.robot.utils.TunableController;
import frc.robot.utils.TunableController.TunableControllerType;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private boolean useVisionAlign = true;

  public static volatile boolean BEFORE_MATCH = false; // Controls MT1-only usage before match

  // Swerve & Subsystem Fields
  private final LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts;
  private final TunableController joystick =
      new TunableController(0).withControllerType(TunableControllerType.QUADRATIC);
  private final TunableController joystick2 =
      new TunableController(1).withControllerType(TunableControllerType.QUADRATIC);

  private final TunableController joystick3 =
      new TunableController(2).withControllerType(TunableControllerType.QUADRATIC);

  // Swerve drive requests and helpers
  private final SwerveRequest.FieldCentric driveRequest =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed.times(0.1))
          .withRotationalDeadband(Constants.MaxAngularRate.times(0.1))
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt pointRequest = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric();
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed.times(0.1))
          .withRotationalDeadband(Constants.MaxAngularRate.times(0.1)) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // Subsystems
  private final Drive drivetrain;
  private final Flywheel flywheel;
  private final Elevator elevator;
  private final Arm arm;
  // Vision subsystem field for non-sim (REAL) mode
  private Vision vision;

  // Additional subsystems & commands
  private shooter shoot = new shooter();
  private elevatorsub elevator1 = new elevatorsub();
  private algee algea = new algee();
  private LEDSubsystem led = new LEDSubsystem();
  private final PhotonVision hi;
  private final ScourceCam cam;
  private climbsub climb = new climbsub();

  // Autonomous chooser
  private final LoggedDashboardChooser<Command> autoChooser;

  // Autonomous command instance
  private Command m_autonomousCommand;

  public Robot() {

    // --- Setup Logging, CAN, & Pathfinding ---
    CanBridge.runTCP();
    Pathfinding.setPathfinder(new LocalADStarAK());

    switch (Constants.currentMode) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case SIM:
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case REPLAY:
        setUseTiming(false); // Run as fast as possible during replay
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }
    Logger.start();
    RobotController.setBrownoutVoltage(6.0);
    FollowPathCommand.warmupCommand().schedule();
    PathfindingCommand.warmupCommand().schedule();

    // --- Initialize Subsystems ---
    DriveIO currentDriveIO = TunerConstants.createDrivetrain();
    switch (Constants.currentMode) {
      case REAL:
        drivetrain = new Drive(currentDriveIO);
        hi = new PhotonVision(drivetrain);

        cam = new ScourceCam(drivetrain);

        // Initialize vision for the real robot using limelight cameras.

        flywheel = new Flywheel(new FlywheelIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        arm = new Arm(new ArmIO() {});
        break;
      case SIM:
        drivetrain = new Drive(currentDriveIO);
        hi = new PhotonVision(drivetrain);
        cam = new ScourceCam(drivetrain);

        flywheel = new Flywheel(new FlywheelIOSIM());
        elevator = new Elevator(new ElevatorIOSIM());
        arm = new Arm(new ArmIOSIM());
        break;
      default:
        drivetrain = new Drive(new DriveIO() {});
        hi = new PhotonVision(drivetrain);

        cam = new ScourceCam(drivetrain);

        vision =
            new Vision(
                drivetrain::addVisionData,
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {});
        flywheel = new Flywheel(new FlywheelIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        arm = new Arm(new ArmIOCTRE() {});

        break;
    }

    NamedCommands.registerCommand("shoot", shoot.autoncmdOut(-0.2, 16));
    NamedCommands.registerCommand("Timeout", shoot.autoncmdIn(11));
    NamedCommands.registerCommand("Intake", shoot.autoncmdIn(0.3));
    NamedCommands.registerCommand("IntakeLong", shoot.autoncmdIn(0.3));

    NamedCommands.registerCommand("Backdrive", shoot.cmd(0.05));

    // OG one below

    // NamedCommands.registerCommand(
    //     "autoalighn",
    //     drivetrain.defer(
    //         () ->
    //             new ParallelCommandGroup(
    //                     drivetrain.autoAlighnTopose(
    //                         SidePoseMatcher.getBackedUpClosestPose(drivetrain.getPose())),
    //                     new Elevatorcmd(elevator1, true))
    //                 .until(
    //                     () ->
    //                         drivetrain.isAtTarget(
    //                                 SidePoseMatcher.getBackedUpClosestPose(drivetrain.getPose()),
    //                                 drivetrain.getPose())
    //                             && elevator1.autoncheck(elevator1.elevatorpos()))
    //                 .andThen(
    //                     drivetrain
    //
    // .autoAlighnTopose(SidePoseMatcher.getClosestPose(drivetrain.getPose()))
    //                         .alongWith(new Elevatorcmd(elevator1, true)))
    //                 .until(
    //                     () ->
    //                         drivetrain.isAtTarget(
    //                                 SidePoseMatcher.getClosestPose(drivetrain.getPose()),
    //                                 drivetrain.getPose())
    //                             && elevator1.autoncheck(elevator1.elevatorpos()))
    //                 .andThen(
    //                     new ParallelCommandGroup(
    //                         new Elevatorcmd(elevator1, true),
    //                         shoot.cmd(-0.3),
    //                         drivetrain.autoAlighnTopose(
    //                             SidePoseMatcher.getClosestPose(drivetrain.getPose()))))
    //                 .until(() -> shoot.hasVelocityautoalighn())
    //                 .andThen(
    //                     new ParallelCommandGroup(
    //                         drivetrain.autoAlighnTopose(
    //                             SidePoseMatcher.getBackedUpClosestPose(drivetrain.getPose())),
    //                         new Elevatorcmd(elevator1, true),
    //                         shoot.cmd(1),
    //                         shoot.runOnce(() -> shoot.visioncheck())))
    //                 .until(
    //                     () ->
    //                         drivetrain.isAtTarget(
    //                                 SidePoseMatcher.getBackedUpClosestPose(drivetrain.getPose()),
    //                                 drivetrain.getPose())
    //                             && shoot.velocitycheck())
    //                 // .until(
    //                 //     () ->
    //                 //         drivetrain.isAtTarget(
    //                 //             SidePoseMatcher.getBackedUpClosestPose(drivetrain.getPose()),
    //                 //             drivetrain.getPose()))))
    //                 .andThen(
    //                     new SequentialCommandGroup(
    //                         elevator1.Motionmagictoggle(0),
    //                         new AutonElevatorcmd(elevator1, 0, false)))));

    // gpt version 1 below error on return

    // NamedCommands.registerCommand(
    //     "autoalign",
    //     drivetrain.defer(
    //         () ->
    //           // Capture target poses once so they remain constant during the command.

    //           return new SequentialCommandGroup(
    //               // Step 1: Drive toward the backup pose with the elevator raised.
    //               new ParallelCommandGroup(
    //                       drivetrain.autoAlighnTopose(backupPose),
    //                       new Elevatorcmd(elevator1, 2, true))
    //                   .until(
    //                       () ->
    //                           drivetrain.isAtTarget(backupPose, drivetrain.getPose())
    //                               && elevator1.autoncheck(2)),

    //               // Step 2: Drive toward the closest pose with the elevator still raised.
    //               new ParallelCommandGroup(
    //                       drivetrain.autoAlighnTopose(closestPose),
    //                       new Elevatorcmd(elevator1, 2, true))
    //                   .until(
    //                       () ->
    //                           drivetrain.isAtTarget(closestPose, drivetrain.getPose())
    //                               && elevator1.autoncheck(2)),

    //               // Step 3: Initiate scoring – spin up the shooter and maintain position.
    //               new ParallelCommandGroup(
    //                       new Elevatorcmd(elevator1, 2, true),
    //                       shoot.cmd(-0.3),
    //                       drivetrain.autoAlighnTopose(closestPose))
    //                   .until(() -> shoot.hasVelocityautoalighn()),

    //               // Step 4: Return to the backup pose while finalizing the shot.
    //               new ParallelCommandGroup(
    //                       drivetrain.autoAlighnTopose(backupPose),
    //                       new Elevatorcmd(elevator1, 2, true),
    //                       shoot.cmd(1),
    //                       shoot.runOnce(() -> shoot.visioncheck()))
    //                   .until(
    //                       () ->
    //                           drivetrain.isAtTarget(backupPose, drivetrain.getPose())
    //                               && shoot.velocitycheck()))),

    //               // Step 5: Reset elevator and stop drivetrain completely to allow the next
    //               // command.
    //               new SequentialCommandGroup(
    //                   elevator1.Motionmagictoggle(0), new Elevatorcmd(elevator1, 0, false)));

    // NamedCommands.registerCommand(
    //     "autoalighn",
    //     drivetrain.defer(
    //         () -> {
    //           // Compute the target poses once.
    //           Pose2d backupPose = SidePoseMatcher.getBackedUpClosestPose(drivetrain.getPose());
    //           Pose2d closestPose = SidePoseMatcher.getClosestPose(drivetrain.getPose());

    //           return new SequentialCommandGroup(
    //               // Step 1: Drive to the backup pose with elevator active.
    //               new ParallelCommandGroup(
    //                       drivetrain.autoAlighnTopose(backupPose),
    //                       new AutonElevatorcmd(elevator1, 2, true))
    //                   .until(
    //                       () ->
    //                           drivetrain.isAtTarget(backupPose, drivetrain.getPose())
    //                               && elevator1.autoncheck(2)),
    //               // Step 2: Drive to the closest pose while keeping the elevator raised.
    //               new ParallelCommandGroup(
    //                       drivetrain.autoAlighnTopose(closestPose),
    //                       new AutonElevatorcmd(elevator1, 2, true))
    //                   .until(
    //                       () ->
    //                           drivetrain.isAtTarget(closestPose, drivetrain.getPose())
    //                               && elevator1.autoncheck(2)),
    //               // Step 3: Spin up the shooter while holding at the closest pose.
    //               new ParallelCommandGroup(
    //                       new AutonElevatorcmd(elevator1, 2, true),
    //                       shoot.cmd(-0.3),
    //                       drivetrain.autoAlighnTopose(closestPose))
    //                   .until(() -> shoot.hasVelocityautoalighn()),
    //               // Step 4: Return to the backup pose to finalize the shot.
    //               new ParallelCommandGroup(
    //                       drivetrain.autoAlighnTopose(backupPose),
    //                       new AutonElevatorcmd(elevator1, 2, true),
    //                       shoot.cmd(1),
    //                       shoot.runOnce(() -> shoot.visioncheck()))
    //                   .until(
    //                       () ->
    //                           drivetrain.isAtTarget(backupPose, drivetrain.getPose())
    //                               && shoot.velocitycheck()),
    //               // Step 5: Stop the drivetrain.
    //               new InstantCommand(drivetrain::stop, drivetrain),
    //               // Step 6: Reset the elevator (wrap in an InstantCommand if needed).
    //               new InstantCommand(() -> elevator1.Motionmagictoggle(0), elevator1),
    //               // Step 7: Lower the elevator.
    //               new AutonElevatorcmd(elevator1, 0, false));
    //         }));

    NamedCommands.registerCommand(
        "autoalighnr",
        drivetrain
            .defer(
                () ->
                    new Elevatorcmd(elevator1, true)
                        .until(() -> elevator1.autoalighncheck(elevator1.elevatorpos()))
                        .andThen(
                            new ParallelCommandGroup(
                                drivetrain.autoAlighnTopose(
                                    SidePoseMatcher.getBackedUpClosestRightPose(
                                        drivetrain.getPose())),
                                new Elevatorcmd(elevator1, true)))
                        .until(
                            () ->
                                drivetrain.isAtTarget(
                                    SidePoseMatcher.getBackedUpClosestRightPose(
                                        drivetrain.getPose()),
                                    drivetrain.getPose()))
                        .andThen(
                            new ParallelCommandGroup(new Elevatorcmd(elevator1, true))
                                .withTimeout(0.3))
                        .andThen(
                            new ParallelCommandGroup(
                                shoot.cmd(-0.2), new Elevatorcmd(elevator1, true))))
            .until(
                () ->
                    elevator1.autoalighncheck(elevator1.elevatorpos())
                        && Math.abs(shoot.velocity()) > 16));

    NamedCommands.registerCommand(
        "autoalighnL",
        drivetrain
            .defer(
                () ->
                    new Elevatorcmd(elevator1, true)
                        .until(() -> elevator1.autoalighncheck(elevator1.elevatorpos()))
                        .andThen(
                            new ParallelCommandGroup(
                                drivetrain.autoAlighnTopose(
                                    SidePoseMatcher.getBackedUpClosestLeftPose(
                                        drivetrain.getPose())),
                                new Elevatorcmd(elevator1, true)))
                        .until(
                            () ->
                                drivetrain.isAtTarget(
                                    SidePoseMatcher.getBackedUpClosestLeftPose(
                                        drivetrain.getPose()),
                                    drivetrain.getPose()))
                        .andThen(
                            new ParallelCommandGroup(new Elevatorcmd(elevator1, true))
                                .withTimeout(0.3))
                        .andThen(
                            new ParallelCommandGroup(
                                shoot.cmd(-0.2), new Elevatorcmd(elevator1, true))))
            .until(
                () ->
                    elevator1.autoalighncheck(elevator1.elevatorpos())
                        && Math.abs(shoot.velocity()) > 16));

    // NamedCommands.registerCommand(
    //     "elevatorupright",
    //     elevator1.defer(
    //         () ->
    //             new AutonElevatorcmd(elevator1, 2, true)
    //                 .until(
    //                     () ->
    //                         SidePoseMatcher.getDistanceToTarget(
    //                                 drivetrain.getPose(),
    //                                 SidePoseMatcher.getBackedUpClosestRightPose(
    //                                     drivetrain.getPose()))
    //                             > 1)
    //                 .andThen(
    //                     new SequentialCommandGroup(
    //                         elevator1.Motionmagictoggle(0),
    //                         new ParallelCommandGroup(new AutonElevatorcmd(elevator1, 0,
    // false))))));

    NamedCommands.registerCommand("elevatoru", new Elevatorcmd(elevator1, true));

    NamedCommands.registerCommand("reset", elevator1.runOnce(() -> elevator1.resetenc()));

    NamedCommands.registerCommand(
        "elevatord",
        new SequentialCommandGroup(
            elevator1.Motionmagictoggle(0),
            new ParallelCommandGroup(new AutonomusElevatorcmd(elevator1, 0, false))));

    NamedCommands.registerCommand(
        "AlgaeCenterl2",
        drivetrain.defer(
            () ->
                new l3algae(algea, -0.7, 5, elevator1, -11.679, 0)
                    .until(() -> elevator1.autoalighncheckpiv(-11.679))
                    .andThen(
                        new ParallelCommandGroup(
                            drivetrain.autoAlighnTopose(
                                SidePoseMatcher.moveforwaord2Meters(
                                    SidePoseMatcher.getCenterReefPose(drivetrain.getPose()))),
                            new l3algae(algea, -0.7, 5, elevator1, -11.679, 0)))));

    NamedCommands.registerCommand(
        "bargecmd",
        new barge(elevator1, 25.44423828125, true, algea)
            .until(() -> elevator1.autoalighncheckclimb(25.44423828125)));

    // --- Setup Autonomous Chooser ---
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // --- Register Named Commands ---

    // --- Configure Joystick Bindings & Default Commands ---
    configureBindings();
  }

  private void configureBindings() {

    // joystick2
    //     .b()
    //     .whileTrue(
    //         elevator1.Motionmagictoggle(2)
    //             .until(() -> elevator1.autoalighncheck(2))
    //             .andThen(elevator1.Flipydo(Constants.l2)));

    Command Positionl2 =

        // // setpoint
        // drivetrain
        //     .defer(
        //         () ->
        //             new l3algae(algea, -0.7, 5, elevator1, -11.679, 0)
        //                 .until(() -> elevator1.autoalighncheckpiv(-11.679))
        //                 .andThen(
        //                     new ParallelCommandGroup(
        //                         drivetrain.autoAlighnTopose(
        //                             SidePoseMatcher.getCenterReefPose(drivetrain.getPose())),
        //                         new l3algae(algea, -0.7, 5, elevator1, -11.679, 0)))
        //                 .andThen(
        //                     new ParallelCommandGroup(
        //                         shoot.cmd(-0.2), new Elevatorcmd(elevator1, true)))
        //                 .until(() -> Math.abs(shoot.velocity()) > 16))
        //     .onlyWhile(() -> Constants.getRobotState() != Constants.RobotState.ALGEA);

        new l3algae(algea, -0.7, 5, elevator1, -11.679, 0);

    Command Positionl21 =

        // setpoint
        new l3algae(algea, -0.7, 5, elevator1, -11.679, 0);

    Command hyper =

        // setpoint

        new Hyper(algea, -0.5, 5, elevator1, -11.679, 0);

    Command hyper1 =

        // setpoint
        new Hyper(algea, -0.5, 5, elevator1, -11.679, 0);
    Command Positionl3 =
        // new SequentialCommandGroup(new l2algae(algea, 0.8, 5, elevator1, -18.31416015625));
        new SequentialCommandGroup(new l3algae(algea, -0.5, 5, elevator1, -14.03251953125, 6.5));

    Command Positionl31 =
        // new SequentialCommandGroup(new l2algae(algea, 0.8, 5, elevator1, -18.31416015625));
        new SequentialCommandGroup(new l3algae(algea, -0.5, 5, elevator1, -14.03251953125, 6.5));

    Command hyperl3 =
        // new SequentialCommandGroup(new l2algae(algea, 0.8, 5, elevator1, -18.31416015625));
        new SequentialCommandGroup(new Hyperl3(algea, -0.5, 5, elevator1, -14.03251953125, 6.5));

    Command hyperl31 =
        // new SequentialCommandGroup(new l2algae(algea, 0.8, 5, elevator1, -18.31416015625));
        new SequentialCommandGroup(new Hyperl3(algea, -0.5, 5, elevator1, -14.03251953125, 6.5));

    // Set up the default swerve drive command.
    // Set up the default swerve drive command.
    // if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
    //   drivetrain.setDefaultCommand(
    //       drivetrain.applyRequest(
    //           () -> {
    //             double x = joystick.customLeft().getX() * 0.9;
    //             double y = joystick.customLeft().getY() * 0.9;
    //             double rot = joystick.customRight().getX() * 0.9;
    //             if (Math.abs(x) < 0.05 && Math.abs(y) < 0.05 && Math.abs(rot) < 0.05) {
    //               return brakeRequest;
    //             } else {
    //               return driveRequest
    //                   .withVelocityX(MaxSpeed.times(-y))
    //                   .withVelocityY(MaxSpeed.times(-x))
    //                   .withRotationalRate(Constants.MaxAngularRate.times(-rot));
    //             }
    //           }));
    // } else {
    //   drivetrain.setDefaultCommand(
    //       drivetrain.applyRequest(
    //           () -> {
    //             double x = joystick.customLeft().getX();
    //             double y = joystick.customLeft().getY();
    //             double rot = joystick.customRight().getX();
    //             if (Math.abs(x) < 0.05 && Math.abs(y) < 0.05 && Math.abs(rot) < 0.05) {
    //               return brakeRequest;
    //             } else {
    //               return driveRequest
    //                   .withVelocityX(MaxSpeed.times(-y))
    //                   .withVelocityY(MaxSpeed.times(-x))
    //                   .withRotationalRate(Constants.MaxAngularRate.times(-rot));
    //             }
    //           }));
    // }

    // joystick.leftBumper().whileTrue(new Lynkalighnmentleft(drivetrain, false, true, 2, 0));

    drivetrain.setDefaultCommand(

        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        MaxSpeed.times(
                            -joystick3.customLeft().getY()
                                * elevator1
                                    .getInvertedElevatorProgress())) // Drive forward with negative
                    // Y (forward)
                    .withVelocityY(
                        MaxSpeed.times(
                            -joystick3.customLeft().getX()
                                * elevator1
                                    .getInvertedElevatorProgress())) // Drive left with negative X
                    .withRotationalRate(
                        Constants.MaxAngularRate.times(
                            -joystick3
                                .customRight()
                                .getX())))); // Drive counterclockwise with negative X
    // (left)
    // Additional joystick bindings for shooter, elevator, etc.

    // intake
    // joystick
    //     .leftTrigger(0.2)
    //     .onTrue(
    //         new SequentialCommandGroup(
    //             // While the button is held, run intake and bring the elevator down.
    //             new ParallelDeadlineGroup(
    //                 // End this group when a current spike indicates a piece is intaken.
    //                 new WaitUntilCommand(() -> shoot.velocitywe()),
    //                 // Run both the intake command and move the elevator to the down setpoint
    // (index
    //                 // 0).
    //                 new ParallelCommandGroup(
    //                     shoot.cmd(0.3),
    //                     new Elevatorcmd(
    //                         elevator1, 0, true) // "0" is assumed to be the down position.
    //                     )),
    //             // Wait until you release the intake button.
    //             new WaitUntilCommand(() -> !joystick.leftTrigger(0.2).getAsBoolean()),
    //             // Once released, move the elevator to L2 (setpoint index 2).
    //             new ParallelCommandGroup(shoot.cmd(0.07), new Elevatorcmd(elevator1, 2, true))));
    // joystick
    //     .leftTrigger(0.2)
    //     .whileTrue(
    //         new ParallelCommandGroup(shoot.cmd(0.3), elevator1.runOnce(() ->
    // elevator1.resetenc())))
    //     .whileFalse(shoot.cmd(0.07));

    // joystick
    //     .rightTrigger(0.2)
    //     .whileTrue(
    //         new ConditionalCommand(
    //             // Condition: if either stick is pressed, execute coral shoot
    //             shoot.cmd(-0.2),
    //             // Else, check if the back button is pressed
    //             new ConditionalCommand(
    //                 // If back button is pressed, then check robot state:
    //                 new ConditionalCommand(
    //                     // If in ALGEA mode, execute algea shoot
    //                     algea.algeacmd(0.7),
    //                     // Otherwise, execute coral shoot
    //                     shoot.cmd(-0.2),
    //                     () -> Constants.getRobotState() == Constants.RobotState.ALGEA),
    //                 // If the back button isn't pressed, default to coral shoot
    //                 shoot.cmd(-0.2),
    //                 () -> joystick.back().getAsBoolean()),
    //             // Outer condition: sticks have priority
    //             () -> joystick.rightStick().getAsBoolean() ||
    // joystick.leftStick().getAsBoolean()))
    //     .whileFalse(shoot.cmd(0.07));

    // joystick
    //     .rightTrigger(0.2)
    //     .whileTrue(
    //         new ConditionalCommand(
    //             shoot.cmd(-0.2),
    //             algea.algeacmd(-0.7),
    //             () -> Constants.getRobotState() != Constants.RobotState.ALGEA))
    //     .whileFalse(
    //         new ParallelCommandGroup(
    //             shoot.cmd(0.05).onlyWhile(() -> joystick.getRightTriggerAxis() < 0.2)));

    // joystick.x().onTrue(drivetrain.runOnce(() -> drivetrain.resetgyro()));
    // joystick
    //     .back()
    //     .whileTrue(
    //         drivetrain.applyRequest(
    //             () ->
    //                 drive
    //                     .withVelocityX(
    //                         MaxSpeed.times(-joystick.customLeft().getY() * 0.75).times(0.5))
    //                     .withVelocityY(
    //                         MaxSpeed.times(-joystick.customLeft().getX() * 0.75).times(0.5))
    //                     .withRotationalRate(
    //                         Constants.MaxAngularRate.times(-joystick.customRight().getX())
    //                             .times(0.5))));

    // // joystick
    // //     .a()
    // //     .whileTrue(new Elevatorcmd(elevator1, 1, true))
    // //     .whileFalse(
    // //         new SequentialCommandGroup(
    // //             elevator1.Motionmagictoggle(0),
    // //             new ParallelCommandGroup(new Elevatorcmd(elevator1, 0, false))));
    // // joystick.a().whileTrue(new PIDSwerve(drivetrain,new Pose2d(1,2,new Rotation2d())));
    // // joystick
    // //     .leftBumper()
    // //     .whileTrue(
    // //         drivetrain.defer(
    // //             () ->
    // //                 drivetrain.autoAlighnTopose(
    // //                     SidePoseMatcher.getClosestPose(drivetrain.getPose()))));

    // // joystick
    // //     .leftBumper()
    // //     .whileTrue(
    // //         drivetrain.defer(
    // //             () ->
    // //                 drivetrain
    // //                     .autoAlighnTopose(
    // //                         SidePoseMatcher.getBackedUpClosestPose(drivetrain.getPose()))
    // //                     .until(
    // //                         () ->
    // //                             drivetrain.isAtTarget(
    // //
    // SidePoseMatcher.getBackedUpClosestPose(drivetrain.getPose()),
    // //                                 drivetrain.getPose()))
    // //                     .andThen(
    // //                         drivetrain
    // //                             .autoAlighnTopose(
    // //                                 SidePoseMatcher.getClosestPose(drivetrain.getPose()))
    // //                             .until(
    // //                                 () ->
    // //                                     drivetrain.isAtTarget(
    // //
    // SidePoseMatcher.getClosestPose(drivetrain.getPose()),
    // //                                         drivetrain.getPose())))
    // //                     .andThen(
    // //                         new ParallelCommandGroup(
    // //                             new Elevatorcmd(elevator1, 2, true),
    // //                             drivetrain.autoAlighnTopose(
    // //                                 SidePoseMatcher.getClosestPose(drivetrain.getPose()))))))
    // //     .whileFalse(
    // //         new SequentialCommandGroup(
    // //             elevator1.Motionmagictoggle(0), new Elevatorcmd(elevator1, 0, false)));

    // joystick
    //     .y()
    //     .whileTrue(new Elevatorcmd(elevator1, 2, true))
    //     .whileFalse(
    //         new SequentialCommandGroup(
    //             elevator1.Motionmagictoggle(0), new Elevatorcmd(elevator1, 0, false)));

    // joystick2
    //     .y()
    //     .whileTrue(new Elevatorcmd(elevator1, 4, true))
    //     .whileFalse(
    //         new SequentialCommandGroup(
    //             elevator1.Motionmagictoggle(0), new Elevatorcmd(elevator1, 0, false)));

    // joystick2
    //     .b()
    //     .whileTrue(new Elevatorcmd(elevator1, 3, true))
    //     .whileFalse(
    //         new SequentialCommandGroup(
    //             elevator1.Motionmagictoggle(0), new Elevatorcmd(elevator1, 0, false)));

    // joystick2
    //     .a()
    //     .whileTrue(new Elevatorcmd(elevator1, 2, true))
    //     .whileFalse(
    //         new SequentialCommandGroup(
    //             elevator1.Motionmagictoggle(0), new Elevatorcmd(elevator1, 0, false)));

    // joystick
    //     .leftTrigger(0.2)
    //     .whileTrue(
    //         new ParallelCommandGroup(
    //             shoot.cmd(0.5),
    //             elevator1.Flipydo(-0.4),
    //             elevator1.runOnce(() -> elevator1.resetenc())))
    //     .whileFalse(
    //         new ParallelCommandGroup(
    //             shoot.cmd(0.05).onlyWhile(() -> joystick.getRightTriggerAxis() < 0.2),
    //             elevator1.Flipydo(Constants.l2)
    //                 .onlyWhile(
    //                     () ->
    //                         !(joystick.rightStick().getAsBoolean()
    //                             | joystick.leftStick().getAsBoolean()
    //                             | joystick.back().getAsBoolean()
    //                             | joystick.getLeftTriggerAxis() > 0.2
    //                             | joystick.rightBumper().getAsBoolean()
    //                             | joystick.leftBumper().getAsBoolean()))));
    // // Command l2 =
    // //     drivetrain
    // //         .defer(
    // //             () ->
    // //                 new ParallelCommandGroup(
    // //                         drivetrain.autoAlighnToposel2(
    // //                             SidePoseMatcher.getBackedUpClosestPose(drivetrain.getPose())),
    // //                         new Elevatorcmd(elevator1, 2, true))
    // //                     .until(
    // //                         () ->
    // //                             drivetrain.isAtTarget(
    // //                                     SidePoseMatcher.getBackedUpClosestPose(
    // //                                         drivetrain.getPose()),
    // //                                     drivetrain.getPose())
    // //                                 && elevator1.autoncheck(2))
    // //                     .andThen(
    // //                         drivetrain
    // //                             .autoAlighnTopose(
    // //                                 SidePoseMatcher.getClosestPose(drivetrain.getPose()))
    // //                             .alongWith(new Elevatorcmd(elevator1, 2, true)))
    // //                     .until(
    // //                         () ->
    // //                             drivetrain.isAtTarget(
    // //                                     SidePoseMatcher.getClosestPose(drivetrain.getPose()),
    // //                                     drivetrain.getPose())
    // //                                 && elevator1.autoncheck(2))
    // //                     .andThen(
    // //                         new ParallelCommandGroup(
    // //                             new Elevatorcmd(elevator1, 2, true),
    // //                             shoot.cmd(-0.1),
    // //                             drivetrain.autoAlighnToposel2(
    // //                                 SidePoseMatcher.getClosestPose(drivetrain.getPose()))))
    // //                     .until(() -> shoot.hasVelocityautoalighn())
    // //                     .andThen(
    // //                         new ParallelCommandGroup(
    // //                             drivetrain.autoAlighnToposel2(
    // //
    // // SidePoseMatcher.getBackedUpClosestPose(drivetrain.getPose())),
    // //                             new Elevatorcmd(elevator1, 2, true)))
    // //                     .until(
    // //                         () ->
    // //                             drivetrain.isAtTarget(
    // //                                     SidePoseMatcher.getBackedUpClosestPose(
    // //                                         drivetrain.getPose()),
    // //                                     drivetrain.getPose())
    // //                                 && Constants.getCoralstate() ==
    // // Constants.getCoralstate().None)
    // //                     // .until(
    // //                     //     () ->
    // //                     //         drivetrain.isAtTarget(
    // //                     //
    // // SidePoseMatcher.getBackedUpClosestPose(drivetrain.getPose()),
    // //                     //             drivetrain.getPose()))))
    // //                     .andThen(
    // //                         new SequentialCommandGroup(
    // //                                 elevator1.Motionmagictoggle(0),
    // //                                 new Elevatorcmd(elevator1, 0, false))
    // //                             .until(() -> elevator1.autoalighncheck(0))))
    // //         .onlyIf(() -> Constants.getRobotState() == Constants.RobotState.IDLE);
    // // Command l3 =
    // //     drivetrain
    // //         .defer(
    // //             () ->
    // //                 new ParallelCommandGroup(
    // //                         drivetrain.autoAlighnToposel3(
    // //                             SidePoseMatcher.getBackedUpClosestPose(drivetrain.getPose())),
    // //                         new Elevatorcmd(elevator1, 3, true))
    // //                     .until(
    // //                         () ->
    // //                             drivetrain.isAtTarget(
    // //                                     SidePoseMatcher.getBackedUpClosestPose(
    // //                                         drivetrain.getPose()),
    // //                                     drivetrain.getPose())
    // //                                 && elevator1.autoncheck(3))
    // //                     .andThen(
    // //                         drivetrain
    // //                             .autoAlighnToposel3(
    // //                                 SidePoseMatcher.getClosestPose(drivetrain.getPose()))
    // //                             .alongWith(new Elevatorcmd(elevator1, 3, true)))
    // //                     .until(
    // //                         () ->
    // //                             drivetrain.isAtTarget(
    // //                                     SidePoseMatcher.getClosestPose(drivetrain.getPose()),
    // //                                     drivetrain.getPose())
    // //                                 && elevator1.autoncheck(3))
    // //                     .andThen(
    // //                         new ParallelCommandGroup(
    // //                             new Elevatorcmd(elevator1, 3, true),
    // //                             shoot.cmd(-0.1),
    // //                             drivetrain.autoAlighnToposel3(
    // //                                 SidePoseMatcher.getClosestPose(drivetrain.getPose()))))
    // //                     .until(() -> shoot.hasVelocityautoalighn())
    // //                     .andThen(
    // //                         new ParallelCommandGroup(
    // //                             drivetrain.autoAlighnToposel3(
    // //
    // // SidePoseMatcher.getBackedUpClosestPose(drivetrain.getPose())),
    // //                             new Elevatorcmd(elevator1, 3, true)))
    // //                     .until(
    // //                         () ->
    // //                             drivetrain.isAtTarget(
    // //                                     SidePoseMatcher.getBackedUpClosestPose(
    // //                                         drivetrain.getPose()),
    // //                                     drivetrain.getPose())
    // //                                 && Constants.getCoralstate() ==
    // // Constants.getCoralstate().None))
    // //         // .until(
    // //         //     () ->
    // //         //         drivetrain.isAtTarget(
    // //         //             SidePoseMatcher.getBackedUpClosestPose(drivetrain.getPose()),
    // //         //             drivetrain.getPose()))))

    // //         .onlyIf(() -> Constants.getRobotState() == Constants.RobotState.IDLE);
    // // Command l4 =
    // //     drivetrain.defer(
    // //         () ->
    // //             new ParallelCommandGroup(
    // //                     drivetrain.autoAlighnTopose(
    // //                         SidePoseMatcher.getBackedUpClosestPose(drivetrain.getPose())),
    // //                     new Elevatorcmd(elevator1, 4, true))
    // //                 .until(
    // //                     () ->
    // //                         drivetrain.isAtTarget(
    // //
    // SidePoseMatcher.getBackedUpClosestPose(drivetrain.getPose()),
    // //                                 drivetrain.getPose())
    // //                             && elevator1.autoncheck(4))
    // //                 .andThen(
    // //                     drivetrain
    // //
    // // .autoAlighnTopose(SidePoseMatcher.getClosestPose(drivetrain.getPose()))
    // //                         .alongWith(new Elevatorcmd(elevator1, 4, true)))
    // //                 .until(
    // //                     () ->
    // //                         drivetrain.isAtTarget(
    // //                                 SidePoseMatcher.getClosestPose(drivetrain.getPose()),
    // //                                 drivetrain.getPose())
    // //                             && elevator1.autoncheck(4))
    // //                 .andThen(
    // //                     new ParallelCommandGroup(
    // //                         new Elevatorcmd(elevator1, 4, true),
    // //                         shoot.cmd(-0.3),
    // //                         drivetrain.autoAlighnTopose(
    // //                             SidePoseMatcher.getClosestPose(drivetrain.getPose()))))
    // //                 .until(() -> shoot.hasVelocityautoalighn())
    // //                 .andThen(
    // //                     new ParallelCommandGroup(
    // //                         drivetrain.autoAlighnTopose(
    // //                             SidePoseMatcher.getBackedUpClosestPose(drivetrain.getPose())),
    // //                         new Elevatorcmd(elevator1, 4, true),
    // //                         shoot.cmd(1)))
    // //                 .until(
    // //                     () ->
    // //                         drivetrain.isAtTarget(
    // //
    // SidePoseMatcher.getBackedUpClosestPose(drivetrain.getPose()),
    // //                                 drivetrain.getPose())
    // //                             && shoot.velocitycheck())
    // //                 // .until(
    // //                 //     () ->
    // //                 //         drivetrain.isAtTarget(
    // //                 //
    // SidePoseMatcher.getBackedUpClosestPose(drivetrain.getPose()),
    // //                 //             drivetrain.getPose()))))
    // //                 .andThen(
    // //                     new SequentialCommandGroup(
    // //                         elevator1.Motionmagictoggle(0), new Elevatorcmd(elevator1, 0,

    // // false))));

    // // Command pathfinding =
    // //     drivetrain.defer(
    // //         () ->
    // //             new ParallelCommandGroup(
    // //                 drivetrain.driveToPose(
    // //                     SidePoseMatcher.getBackedUpClosestPose(drivetrain.getPose()))));

    // Command l2 =
    //     drivetrain
    //         .defer(
    //             () ->
    //                 new ParallelCommandGroup(
    //                         drivetrain.autoAlighnToposel3(
    //                             SidePoseMatcher.getBackedUpClosestPose(drivetrain.getPose())),
    //                         new Elevatorcmd(elevator1, 2, true))
    //                     .until(
    //                         () ->
    //                             drivetrain.isAtTarget(
    //                                     SidePoseMatcher.getBackedUpClosestPose(
    //                                         drivetrain.getPose()),
    //                                     drivetrain.getPose())
    //                                 && elevator1.autoncheck(2))
    //                     .andThen(
    //                         drivetrain
    //                             .autoAlighnToposel3(
    //                                 SidePoseMatcher.getClosestPose(drivetrain.getPose()))
    //                             .alongWith(new Elevatorcmd(elevator1, 2, true)))
    //                     .until(
    //                         () ->
    //                             drivetrain.isAtTarget(
    //                                     SidePoseMatcher.getClosestPose(drivetrain.getPose()),
    //                                     drivetrain.getPose())
    //                                 && elevator1.autoncheck(2))
    //                     .andThen(
    //                         new ParallelCommandGroup(
    //                             new Elevatorcmd(elevator1, 2, true),
    //                             shoot.cmd(-0.2),
    //                             drivetrain.autoAlighnToposel3(
    //                                 SidePoseMatcher.getClosestPose(drivetrain.getPose()))))
    //                     .until(() -> shoot.hasVelocityautoalighn())
    //                     .andThen(
    //                         new ParallelCommandGroup(
    //                             drivetrain.autoAlighnToposel3(
    //
    // SidePoseMatcher.getBackedUpClosestPose(drivetrain.getPose())),
    //                             new Elevatorcmd(elevator1, 2, true),
    //                             shoot.cmd(-0.4)))
    //                     .until(
    //                         () ->
    //                             drivetrain.isAtTarget(
    //                                     SidePoseMatcher.getBackedUpClosestPose(
    //                                         drivetrain.getPose()),
    //                                     drivetrain.getPose())
    //                                 && shoot.velocitycheck())
    //                     // .until(
    //                     //     () ->
    //                     //         drivetrain.isAtTarget(
    //                     //
    // SidePoseMatcher.getBackedUpClosestPose(drivetrain.getPose()),
    //                     //             drivetrain.getPose()))))
    //                     .andThen(
    //                         new SequentialCommandGroup(
    //                             elevator1.Motionmagictoggle(0),
    //                             new Elevatorcmd(elevator1, 0, false))))
    //         .onlyIf(() -> Constants.getRobotState() == Constants.RobotState.IDLE);
    // // Command l3 =
    // //     drivetrain
    // //         .defer(
    // //             () ->
    // //                 new ParallelCommandGroup(
    // //                         drivetrain.autoAlighnToposel3(
    // //                             SidePoseMatcher.getBackedUpClosestPose(drivetrain.getPose())),
    // //                         new Elevatorcmd(elevator1, 3, true))
    // //                     .until(
    // //                         () ->
    // //                             drivetrain.isAtTarget(
    // //                                     SidePoseMatcher.getBackedUpClosestPose(
    // //                                         drivetrain.getPose()),
    // //                                     drivetrain.getPose())
    // //                                 && elevator1.autoncheck(3))
    // //                     .andThen(
    // //                         drivetrain
    // //                             .autoAlighnToposel3(
    // //                                 SidePoseMatcher.getClosestPose(drivetrain.getPose()))
    // //                             .alongWith(new Elevatorcmd(elevator1, 3, true)))
    // //                     .until(
    // //                         () ->
    // //                             drivetrain.isAtTarget(
    // //                                     SidePoseMatcher.getClosestPose(drivetrain.getPose()),
    // //                                     drivetrain.getPose())
    // //                                 && elevator1.autoncheck(3))
    // //                     .andThen(
    // //                         new ParallelCommandGroup(
    // //                             new Elevatorcmd(elevator1, 3, true),
    // //                             shoot.cmd(-0.2),
    // //                             drivetrain.autoAlighnToposel3(
    // //                                 SidePoseMatcher.getClosestPose(drivetrain.getPose()))))
    // //                     .until(() -> shoot.hasVelocityautoalighn())
    // //                     .andThen(
    // //                         new ParallelCommandGroup(
    // //                             drivetrain.autoAlighnToposel3(
    // //
    // // SidePoseMatcher.getBackedUpClosestPose(drivetrain.getPose())),
    // //                             new Elevatorcmd(elevator1, 3, true),
    // //                             shoot.cmd(0.4)))
    // //                     .until(
    // //                         () ->
    // //                             drivetrain.isAtTarget(
    // //                                     SidePoseMatcher.getBackedUpClosestPose(
    // //                                         drivetrain.getPose()),
    // //                                     drivetrain.getPose())
    // //                                 && shoot.velocitycheck())
    // //                     // .until(
    // //                     //     () ->
    // //                     //         drivetrain.isAtTarget(
    // //                     //
    // // SidePoseMatcher.getBackedUpClosestPose(drivetrain.getPose()),
    // //                     //             drivetrain.getPose()))))
    // //                     .andThen(
    // //                         new SequentialCommandGroup(
    // //                             elevator1.Motionmagictoggle(0),
    // //                             new Elevatorcmd(elevator1, 0, false))))
    // //         .onlyIf(() -> Constants.getRobotState() == Constants.RobotState.IDLE);

    // Command l3 =
    //     drivetrain
    //         .defer(
    //             () ->
    //                 new ParallelCommandGroup(
    //                         drivetrain.autoAlighnToposel3(
    //                             SidePoseMatcher.getBackedUpClosestPose(drivetrain.getPose())),
    //                         new Elevatorcmd(elevator1, 3, true))
    //                     .until(
    //                         () ->
    //                             drivetrain.isAtTarget(
    //                                     SidePoseMatcher.getBackedUpClosestPose(
    //                                         drivetrain.getPose()),
    //                                     drivetrain.getPose())
    //                                 && elevator1.autoncheck(3))
    //                     .andThen(
    //                         drivetrain
    //                             .autoAlighnToposel3(
    //                                 SidePoseMatcher.getClosestPose(drivetrain.getPose()))
    //                             .alongWith(new Elevatorcmd(elevator1, 3, true)))
    //                     .until(
    //                         () ->
    //                             drivetrain.isAtTarget(
    //                                     SidePoseMatcher.getClosestPose(drivetrain.getPose()),
    //                                     drivetrain.getPose())
    //                                 && elevator1.autoncheck(3))
    //                     .andThen(
    //                         new ParallelCommandGroup(
    //                             new Elevatorcmd(elevator1, 3, true),
    //                             shoot.cmd(-0.2),
    //                             drivetrain.autoAlighnToposel3(
    //                                 SidePoseMatcher.getClosestPose(drivetrain.getPose()))))
    //                     .until(() -> shoot.hasVelocityautoalighn())
    //                     .andThen(
    //                         new ParallelCommandGroup(
    //                             drivetrain.autoAlighnToposel3(
    //
    // SidePoseMatcher.getBackedUpClosestPose(drivetrain.getPose())),
    //                             new Elevatorcmd(elevator1, 3, true),
    //                             shoot.cmd(-0.4)))
    //                     .until(
    //                         () ->
    //                             drivetrain.isAtTarget(
    //                                     SidePoseMatcher.getBackedUpClosestPose(
    //                                         drivetrain.getPose()),
    //                                     drivetrain.getPose())
    //                                 && shoot.velocitycheck())
    //                     // .until(
    //                     //     () ->
    //                     //         drivetrain.isAtTarget(
    //                     //
    // SidePoseMatcher.getBackedUpClosestPose(drivetrain.getPose()),
    //                     //             drivetrain.getPose()))))
    //                     .andThen(
    //                         new SequentialCommandGroup(
    //                             elevator1.Motionmagictoggle(0),
    //                             new Elevatorcmd(elevator1, 0, false))))
    //         .onlyIf(() -> Constants.getRobotState() == Constants.RobotState.IDLE);

    // Command l4 =
    //     drivetrain
    //         .defer(
    //             () ->
    //                 new ParallelCommandGroup(
    //                         drivetrain.autoAlighnTopose(
    //                             SidePoseMatcher.getBackedUpClosestPose(drivetrain.getPose())),
    //                         new Elevatorcmd(elevator1, 4, true))
    //                     .until(
    //                         () ->
    //                             drivetrain.isAtTarget(
    //                                     SidePoseMatcher.getBackedUpClosestPose(
    //                                         drivetrain.getPose()),
    //                                     drivetrain.getPose())
    //                                 && elevator1.autoncheck(4))
    //                     .andThen(
    //                         drivetrain
    //                             .autoAlighnTopose(
    //                                 SidePoseMatcher.getClosestPose(drivetrain.getPose()))
    //                             .alongWith(new Elevatorcmd(elevator1, 4, true)))
    //                     .until(
    //                         () ->
    //                             drivetrain.isAtTarget(
    //                                     SidePoseMatcher.getClosestPose(drivetrain.getPose()),
    //                                     drivetrain.getPose())
    //                                 && elevator1.autoncheck(4))
    //                     .andThen(
    //                         new ParallelCommandGroup(
    //                             new Elevatorcmd(elevator1, 4, true),
    //                             shoot.cmd(-0.3),
    //                             drivetrain.autoAlighnTopose(
    //                                 SidePoseMatcher.getClosestPose(drivetrain.getPose()))))
    //                     .until(() -> shoot.hasVelocityautoalighn())
    //                     .andThen(
    //                         new ParallelCommandGroup(
    //                             drivetrain.autoAlighnTopose(
    //
    // SidePoseMatcher.getBackedUpClosestPose(drivetrain.getPose())),
    //                             new Elevatorcmd(elevator1, 4, true),
    //                             shoot.cmd(1)))
    //                     .until(
    //                         () ->
    //                             drivetrain.isAtTarget(
    //                                     SidePoseMatcher.getBackedUpClosestPose(
    //                                         drivetrain.getPose()),
    //                                     drivetrain.getPose())
    //                                 && shoot.velocitycheck())
    //                     // .until(
    //                     //     () ->
    //                     //         drivetrain.isAtTarget(
    //                     //
    // SidePoseMatcher.getBackedUpClosestPose(drivetrain.getPose()),
    //                     //             drivetrain.getPose()))))
    //                     .andThen(
    //                         new SequentialCommandGroup(
    //                             elevator1.Motionmagictoggle(0),
    //                             new Elevatorcmd(elevator1, 0, false))))
    //         .onlyIf(() -> Constants.getRobotState() == Constants.RobotState.IDLE);

    // joystick
    //     .pov(0)
    //     .whileTrue(
    //         drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.8).withVelocityY(0)));

    // joystick
    //     .pov(180)
    //     .whileTrue(
    //         drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.8).withVelocityY(0)));
    // joystick
    //     .pov(90)
    //     .whileTrue(
    //         drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0).withVelocityY(-0.8)));

    // joystick
    //     .pov(270)
    //     .whileTrue(
    //         drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0).withVelocityY(0.8)));

    // // joystick
    // //     .rightStick()
    // //     .whileTrue(
    // //         new ConditionalCommand(
    // //             l2, Positionl2, () -> Constants.getRobotState() !=
    // Constants.RobotState.ALGEA))
    // //     .whileFalse(
    // //         new SequentialCommandGroup(
    // //             elevator1.Motionmagictoggle(0), new Elevatorcmd(elevator1, 0, false)));

    // // joystick
    // //     .rightStick()
    // //     .whileTrue(
    // //         new ConditionalCommand(
    // //             new Elevatorcmd(elevator1, 2, true),
    // //             Positionl2, // algae
    // //             () -> Constants.getRobotState() != Constants.RobotState.ALGEA))
    // //     .whileFalse(
    // //         new SequentialCommandGroup(
    // //             elevator1.Motionmagictoggle(0), new Elevatorcmd(elevator1, 0, false)));

    // joystick
    //     .rightStick()
    //     .whileTrue(
    //         new ConditionalCommand(
    //             new ConditionalCommand(
    //                 l2,
    //                 new Elevatorcmd(elevator1, 2, true),
    //                 () -> Constants.getvisionstate() == Constants.autovision.Holding),
    //             Positionl2,
    //             () -> Constants.getRobotState() != Constants.RobotState.ALGEA));

    // // joystick
    // //     .rightStick()
    // //     .whileTrue(
    // //         (l2)
    // //             .unless(
    // //                 () ->
    // //                     Constants.getRobotState() != Constants.RobotState.ALGEA
    // //                         && Constants.getvisionstate() == Constants.autovision.Holding))
    // //     .whileFalse(
    // //         new SequentialCommandGroup(
    // //             elevator1.Motionmagictoggle(0), new Elevatorcmd(elevator1, 0, false)));

    // // joystick
    // //     .rightStick()
    // //     .whileTrue(
    // //         (l2)
    // //             .onlyWhile(
    // //                 () ->
    // //                     Constants.getRobotState() != Constants.RobotState.ALGEA
    // //                         && Constants.getvisionstate() == Constants.autovision.Holding))
    // //     .whileFalse(
    // //         new SequentialCommandGroup(
    // //             elevator1.Motionmagictoggle(0), new Elevatorcmd(elevator1, 0, false)));
    // // joystick
    // //     .rightStick()
    // //     .whileTrue(
    // //         (Positionl2).onlyWhile(() -> Constants.getRobotState() ==
    // // Constants.RobotState.ALGEA))
    // //     .whileFalse(
    // //         new SequentialCommandGroup(
    // //             elevator1.Motionmagictoggle(0), new Elevatorcmd(elevator1, 0, false)));

    // // joystick
    // //     .rightStick()
    // //     .whileTrue(
    // //         new Elevatorcmd(elevator1, 2, true)
    // //             .onlyWhile(
    // //                 () ->
    // //                     Constants.getRobotState() != Constants.RobotState.ALGEA
    // //                         && Constants.getvisionstate() == Constants.autovision.None))
    // //     .whileFalse(
    // //         new ParallelCommandGroup(new Elevatorcmd(elevator1, 2, true),
    // climb.cmdspeed(-0.7)));

    // // joystick
    // //     .rightStick()
    // //     .whileTrue(
    // //         Positionl2.onlyWhile(
    // //             () -> Constants.getRobotState() == Constants.RobotState.ALGEA)) // algae
    // //     .whileFalse(
    // //         new SequentialCommandGroup(
    // //             elevator1.Motionmagictoggle(0), new Elevatorcmd(elevator1, 0, false)));

    // joystick
    //     .leftStick()
    //     .whileTrue(
    //         new ConditionalCommand(
    //             new ConditionalCommand(
    //                 l3,
    //                 new Elevatorcmd(elevator1, 3, true),
    //                 () -> Constants.getvisionstate() == Constants.autovision.Holding),
    //             Positionl3,
    //             () -> Constants.getRobotState() != RobotState.ALGEA));

    // // joystick
    // //     .leftStick()
    // //     .whileTrue(
    // //         (l3)
    // //             .onlyWhile(
    // //                 () ->
    // //                     Constants.getvisionstate() == Constants.autovision.Holding
    // //                         && Constants.getvisionstate() == Constants.autovision.Holding))
    // //     .whileFalse(
    // //         new SequentialCommandGroup(
    // //             elevator1.Motionmagictoggle(0), new Elevatorcmd(elevator1, 0, false)));

    // // joystick
    // //     .leftStick()
    // //     .whileTrue(
    // //         Positionl3.onlyWhile(
    // //             () -> Constants.getRobotState() == Constants.RobotState.ALGEA)) // algae
    // //     .whileFalse(
    // //         new SequentialCommandGroup(
    // //             elevator1.Motionmagictoggle(0), new Elevatorcmd(elevator1, 0, false)));

    // // joystick2.a().whileTrue(shoot.cmd(-0.2));

    // // joystick
    // //     .rightStick()
    // //     .and(joystick.rightTrigger(0.2))
    // //     .whileTrue(shoot.cmd(-0.2))
    // //     .whileFalse(shoot.cmd(0.1));
    // // joystick
    // //     .rightStick()
    // //     .whileTrue(new Elevatorcmd(elevator1, 2, true))
    // //     .whileFalse(
    // //         new SequentialCommandGroup(
    // //             elevator1.Motionmagictoggle(0), new Elevatorcmd(elevator1, 0, false)));

    // // joystick.leftStick().whileTrue(l3) .whileFalse(
    // //     new SequentialCommandGroup(
    // //         elevator1.Motionmagictoggle(0), new Elevatorcmd(elevator1, 0, false)));

    // // joystick
    // //     .leftStick()
    // //     .whileTrue(
    // //         new ConditionalCommand(
    // //             l3,
    // //             new ConditionalCommand(
    // //                 hyperl3.until(()-> elevator1.autoalighncheck(3)).andThen(l3),
    // //                 Positionl3,
    // //                 () -> Constants.getCoralstate() == Constants.coralstate.Holding),
    // //             () -> Constants.getRobotState() != Constants.RobotState.ALGEA))
    // //     .whileFalse(
    // //         new SequentialCommandGroup(
    // //             elevator1.Motionmagictoggle(0),
    // //             new ParallelCommandGroup(new Elevatorcmd(elevator1, 0, false))));

    // joystick.a().whileTrue(elevator1.runOnce(() -> elevator1.togglevision()));

    // // joystick
    // //     .leftStick()
    // //     .whileTrue(
    // //         new ConditionalCommand(
    // //             // If not ALGEA mode → check vision toggle

    // //             l3, // Vision-based command

    // //             // If in ALGEA mode → check coral holding
    // //             new ConditionalCommand(
    // //                 hyperl3,
    // //                 Positionl3,
    // //                 () -> Constants.getCoralstate() == Constants.coralstate.Holding),
    // //             () -> Constants.getRobotState() != Constants.RobotState.ALGEA // Outer
    // condition
    // //             ))
    // //     .whileFalse(
    // //         new SequentialCommandGroup(
    // //             elevator1.Motionmagictoggle(0),
    // //             new ParallelCommandGroup(new Elevatorcmd(elevator1, 0, false))));

    // // joystick
    // //     .leftStick()
    // //     .whileTrue(
    // //         new SequentialCommandGroup(new l3algae(algea, -0.5, 5, elevator1, -15.03251953125,
    // // 5.9))
    // //             .onlyWhile(() -> Constants.getRobotState() == Constants.RobotState.ALGEA))
    // //     .whileFalse(
    // //         new SequentialCommandGroup(
    // //             elevator1.Motionmagictoggle(0), new Elevatorcmd(elevator1, 0, false)));

    // // joystick
    // //     .leftStick()
    // //     .whileTrue(
    // //         new Elevatorcmd(elevator1, 3, true)
    // //             .onlyWhile(() -> Constants.getRobotState() == Constants.RobotState.IDLE))
    // //     .whileFalse(
    // //         new SequentialCommandGroup(
    // //             elevator1.Motionmagictoggle(0), new Elevatorcmd(elevator1, 0, false)));

    // // ;

    // // joystick
    // //     .leftStick()
    // //     .whileTrue(
    // //         new ConditionalCommand(
    // //             new Elevatorcmd(elevator1, 3, true),
    // //             Positionl3,
    // //             () -> Constants.getRobotState() != Constants.RobotState.ALGEA))
    // //     .whileFalse(
    // //         new SequentialCommandGroup(
    // //             elevator1.Motionmagictoggle(0),
    // //             new ParallelCommandGroup(new Elevatorcmd(elevator1, 0, false))));

    // joystick
    //     .back()
    //     .whileTrue(
    //         new ConditionalCommand(
    //             new Elevatorcmd(elevator1, 4, true),
    //             new barge(elevator1, 25.44423828125, true, algea),
    //             () -> Constants.getRobotState() != Constants.RobotState.ALGEA))
    //     .whileFalse(
    //         new SequentialCommandGroup(
    //             elevator1.Motionmagictoggle(0), new Elevatorcmd(elevator1, 0, false)));

    // // joystick.back().whileTrue(l4) .whileFalse(
    // //     new SequentialCommandGroup(
    // //         elevator1.Motionmagictoggle(0), new Elevatorcmd(elevator1, 0, false)));

    // // joystick
    // //     .rightBumper()
    // //     .whileTrue(
    // //         drivetrain.defer(
    // //             () ->
    // //                 drivetrain
    // //                     .driveToPose(SidePoseMatcher.getClosestPose(drivetrain.getPose()))
    // //                     .until(() -> shoot.hasVelocityautoalighn())
    // //                     .andThen(
    // //                         drivetrain.driveToPose(
    // //                             SidePoseMatcher.moveBackward2Meters(drivetrain.getPose())))));

    // // joystick
    // //     .leftBumper()
    // //     .whileTrue(
    // //         new Elevatorcmd(elevator1, 2, true)
    // //             .until(() -> elevator1.autoalighncheck(2))
    // //             .andThen(
    // //                 new ParallelCommandGroup(
    // //                     new Elevatorcmd(elevator1, 2, true), climb.cmdspeed(-0.7))))
    // //     .whileFalse(climb.cmdspeed(0));
    // joystick
    //     .rightBumper()
    //     .whileTrue(
    //         new climbt(elevator1, 3, true)
    //             .until(() -> elevator1.autoalighncheckclimb(0))
    //             .andThen(
    //                 new ParallelCommandGroup(new climbt(elevator1, 2, true), climb.cmdspeed(1))))
    //     .whileFalse(climb.cmdspeed(0));

    // joystick
    //     .leftBumper()
    //     .whileTrue(
    //         new climbt(elevator1, 3, true)
    //             .until(() -> elevator1.autoalighncheckclimb(0))
    //             .andThen(
    //                 new ParallelCommandGroup(new climbt(elevator1, 2, true),
    // climb.cmdspeed(-1))))
    //     .whileFalse(climb.cmdspeed(0));

    // joystick.start().whileTrue(elevator1.runOnce(() -> elevator1.togglesetpoint()));

    // Command l2leftpath =
    //     drivetrain.defer(
    //         () ->
    //             drivetrain.drivetopose(
    //                 SidePoseMatcher.getBackedUpClosestLeftPose(drivetrain.getPose())));

    Command l2right =
        drivetrain
            .defer(
                () ->
                    new Elevatorcmd(elevator1, true)
                        .until(() -> elevator1.autoalighncheck(elevator1.elevatorpos()))
                        .andThen(
                            new ParallelCommandGroup(
                                drivetrain.autoAlighnTopose(
                                    SidePoseMatcher.getBackedUpClosestRightPosec(
                                        drivetrain.getPose())),
                                new Elevatorcmd(elevator1, true)))
                        .until(
                            () ->
                                drivetrain.isAtTarget(
                                    SidePoseMatcher.getBackedUpClosestRightPosec(
                                        drivetrain.getPose()),
                                    drivetrain.getPose()))
                        .andThen(
                            new ParallelCommandGroup(new Elevatorcmd(elevator1, true))
                                .withTimeout(0.3))
                        .andThen(
                            new ParallelCommandGroup(
                                shoot.cmd(-0.2), new Elevatorcmd(elevator1, true))))
            .until(
                () ->
                    elevator1.autoalighncheck(elevator1.elevatorpos())
                        && Math.abs(shoot.velocity()) > 16)
            .onlyWhile(() -> Constants.getRobotState() != Constants.RobotState.ALGEA);

    Command l2rightauto =
        drivetrain.defer(
            () ->
                new Elevatorcmd(elevator1, true)
                    .until(() -> elevator1.autoalighncheck(elevator1.elevatorpos()))
                    .andThen(
                        new ParallelCommandGroup(
                            drivetrain.autoAlighnTopose(
                                SidePoseMatcher.getBackedUpClosestRightPosec(drivetrain.getPose())),
                            new Elevatorcmd(elevator1, true)))
                    .until(
                        () ->
                            drivetrain.isAtTarget(
                                SidePoseMatcher.getBackedUpClosestRightPosec(drivetrain.getPose()),
                                drivetrain.getPose()))
                    .andThen(
                        new ParallelCommandGroup(
                            new Elevatorcmd(elevator1, true),
                            drivetrain.autoAlighnTopose(
                                SidePoseMatcher.getBackedUpClosestRightPose(drivetrain.getPose()))))
                    .withTimeout(0.5)
                    .andThen(
                        new ParallelCommandGroup(shoot.cmd(-0.2), new Elevatorcmd(elevator1, true)))
                    .until(() -> Math.abs(shoot.velocity()) > 18));
    // ;
    Command l2left =
        drivetrain
            .defer(
                () ->
                    new Elevatorcmd(elevator1, true)
                        .until(() -> elevator1.autoalighncheck(elevator1.elevatorpos()))
                        .andThen(
                            new ParallelCommandGroup(
                                drivetrain.autoAlighnTopose(
                                    SidePoseMatcher.getBackedUpClosestLeftPosec(
                                        drivetrain.getPose())),
                                new Elevatorcmd(elevator1, true)))
                        .until(
                            () ->
                                drivetrain.isAtTarget(
                                    SidePoseMatcher.getBackedUpClosestLeftPosec(
                                        drivetrain.getPose()),
                                    drivetrain.getPose()))
                        .andThen(
                            new ParallelCommandGroup(new Elevatorcmd(elevator1, true))
                                .withTimeout(0.3))
                        .andThen(
                            new ParallelCommandGroup(
                                shoot.cmd(-0.2), new Elevatorcmd(elevator1, true))))
            .until(
                () ->
                    elevator1.autoalighncheck(elevator1.elevatorpos())
                        && Math.abs(shoot.velocity()) > 16);

    // intake
    joystick3
        .leftTrigger(0.2)
        .and(joystick.rightStick())
        .whileTrue(
            new ParallelCommandGroup(
                shoot.cmd(0.5),
                elevator1.Flipydo(-0.4),
                elevator1.runOnce(() -> elevator1.resetenc())))
        .whileFalse(
            new ParallelCommandGroup(
                shoot.cmd(0.1).onlyWhile(() -> joystick.getRightTriggerAxis() < 0.2),
                // Use a ConditionalCommand to select the pivot value
                new ConditionalCommand(
                        elevator1.Flipydo(
                            -23.38310546875), // runs when condition is true: elevator in pos 4
                        elevator1.Flipydo(Constants.l2), // otherwise, use the default value
                        () -> elevator1.elevatorpos() == 4 // condition to check elevator position
                        )
                    .onlyWhile(
                        () ->
                            !(joystick.rightStick().getAsBoolean()
                                || joystick.getLeftTriggerAxis() > 0.2
                                || joystick.rightBumper().getAsBoolean()
                                || joystick.leftBumper().getAsBoolean()))));

    joystick3
        .leftTrigger(0.2)
        .whileTrue(
            new ParallelCommandGroup(
                shoot.cmd(0.5),
                elevator1.Flipydo(-0.4),
                elevator1.runOnce(() -> elevator1.resetenc())))
        .whileFalse(
            new ParallelCommandGroup(
                shoot.cmd(0.1).onlyWhile(() -> joystick.getRightTriggerAxis() < 0.2),
                // Use a ConditionalCommand to select the pivot value
                new ConditionalCommand(
                        elevator1.Flipydo(
                            -23.38310546875), // runs when condition is true: elevator in pos 4
                        elevator1.Flipydo(Constants.l2), // otherwise, use the default value
                        () -> elevator1.elevatorpos() == 4 // condition to check elevator position
                        )
                    .onlyWhile(
                        () ->
                            !(joystick.rightStick().getAsBoolean()
                                || joystick.getLeftTriggerAxis() > 0.2
                                || joystick.rightBumper().getAsBoolean()
                                || joystick.leftBumper().getAsBoolean()))));

    joystick3
        .leftTrigger(0.2)
        .and(joystick.leftStick())
        .whileTrue(
            new ParallelCommandGroup(
                shoot.cmd(0.5),
                elevator1.Flipydo(-0.4),
                elevator1.runOnce(() -> elevator1.resetenc())))
        .whileFalse(
            new ParallelCommandGroup(
                shoot.cmd(0.1).onlyWhile(() -> joystick.getRightTriggerAxis() < 0.2),
                // Use a ConditionalCommand to select the pivot value
                new ConditionalCommand(
                        elevator1.Flipydo(
                            -23.38310546875), // runs when condition is true: elevator in pos 4
                        elevator1.Flipydo(Constants.l2), // otherwise, use the default value
                        () -> elevator1.elevatorpos() == 4 // condition to check elevator position
                        )
                    .onlyWhile(
                        () ->
                            !(joystick.rightStick().getAsBoolean()
                                || joystick.getLeftTriggerAxis() > 0.2
                                || joystick.rightBumper().getAsBoolean()
                                || joystick.leftBumper().getAsBoolean()))));

    joystick3
        .leftTrigger(0.2)
        .and(joystick.leftStick())
        .whileTrue(
            new ParallelCommandGroup(
                shoot.cmd(0.5),
                elevator1.Flipydo(-0.4),
                elevator1.runOnce(() -> elevator1.resetenc())))
        .whileFalse(
            new ParallelCommandGroup(
                shoot.cmd(0.1).onlyWhile(() -> joystick.getRightTriggerAxis() < 0.2),
                // Use a ConditionalCommand to select the pivot value
                new ConditionalCommand(
                        elevator1.Flipydo(
                            -23.38310546875), // runs when condition is true: elevator in pos 4
                        elevator1.Flipydo(Constants.l2), // otherwise, use the default value
                        () -> elevator1.elevatorpos() == 4 // condition to check elevator position
                        )
                    .onlyWhile(
                        () ->
                            !(joystick.rightStick().getAsBoolean()
                                || joystick.getLeftTriggerAxis() > 0.2
                                || joystick.rightBumper().getAsBoolean()
                                || joystick.leftBumper().getAsBoolean()))));
    // Outake
    joystick3
        .rightTrigger(0.2)
        .whileTrue(
            new ConditionalCommand(
                shoot.cmd(-0.2),
                new SequentialCommandGroup(
                    elevator1.Flipydo(-0.4).until(() -> elevator1.autoalighncheckpiv(-0.4)),
                    new ParallelCommandGroup(elevator1.Flipydo(-0.4), algea.algeacmd(0.3))),
                () -> Constants.getRobotState() != Constants.RobotState.ALGEA))
        .whileFalse(
            new ParallelCommandGroup(
                shoot.cmd(0.1).onlyWhile(() -> joystick.getRightTriggerAxis() < 0.2)));

    // left Pole/ Algae
    // ─── Left Bumper ───────────────────────────────────────────────────────────────
    joystick3
        .leftBumper()
        .whileTrue(
            new ConditionalCommand(
                // NOT ALGEA → go to l2left
                l2left,
                // ALGEA MODE → check pos1, then pos4, then pos2/3
                new ConditionalCommand(
                    // pos1 → pivot to –7
                    new ParallelCommandGroup(
                        drivetrain.applyRequest(
                            () ->
                                drive
                                    .withVelocityX(MaxSpeed.times(-joystick3.customLeft().getY()))
                                    .withVelocityY(MaxSpeed.times(-joystick3.customLeft().getX()))
                                    .withRotationalRate(
                                        Constants.MaxAngularRate.times(
                                            -joystick3.customRight().getX()))),
                        new l3algae(algea, -0.7, 5, elevator1, -0.4, 0)),
                    // else → check pos4
                    new ConditionalCommand(
                        // pos4 → pivot to 25.444…
                        new ParallelCommandGroup(
                            drivetrain.applyRequest(
                                () ->
                                    drive
                                        .withVelocityX(
                                            MaxSpeed.times(-joystick3.customLeft().getY()))
                                        .withVelocityY(
                                            MaxSpeed.times(-joystick3.customLeft().getX()))
                                        .withRotationalRate(
                                            Constants.MaxAngularRate.times(
                                                -joystick3.customRight().getX()))),
                            new barge(elevator1, 25.44423828125, true, algea)),
                        // else → your existing pos2/3 logic
                        new ConditionalCommand(
                            // pos2 branch
                            new ConditionalCommand(
                                new ParallelCommandGroup(
                                    hyper,
                                    drivetrain.applyRequest(
                                        () ->
                                            drive
                                                .withVelocityX(
                                                    MaxSpeed.times(-joystick3.customLeft().getY()))
                                                .withVelocityY(
                                                    MaxSpeed.times(-joystick3.customLeft().getX()))
                                                .withRotationalRate(
                                                    Constants.MaxAngularRate.times(
                                                        -joystick3.customRight().getX())))),
                                new ParallelCommandGroup(
                                    Positionl2,
                                    drivetrain.applyRequest(
                                        () ->
                                            drive
                                                .withVelocityX(
                                                    MaxSpeed.times(-joystick3.customLeft().getY()))
                                                .withVelocityY(
                                                    MaxSpeed.times(-joystick3.customLeft().getX()))
                                                .withRotationalRate(
                                                    Constants.MaxAngularRate.times(
                                                        -joystick3.customRight().getX())))),
                                () -> Constants.getCoralstate() == Constants.coralstate.Holding),
                            // pos3 branch
                            new ConditionalCommand(
                                new ParallelCommandGroup(
                                    hyperl3,
                                    drivetrain.applyRequest(
                                        () ->
                                            drive
                                                .withVelocityX(
                                                    MaxSpeed.times(-joystick3.customLeft().getY()))
                                                .withVelocityY(
                                                    MaxSpeed.times(-joystick3.customLeft().getX()))
                                                .withRotationalRate(
                                                    Constants.MaxAngularRate.times(
                                                        -joystick3.customRight().getX())))),
                                new ParallelCommandGroup(
                                    Positionl3,
                                    drivetrain.applyRequest(
                                        () ->
                                            drive
                                                .withVelocityX(
                                                    MaxSpeed.times(-joystick3.customLeft().getY()))
                                                .withVelocityY(
                                                    MaxSpeed.times(-joystick3.customLeft().getX()))
                                                .withRotationalRate(
                                                    Constants.MaxAngularRate.times(
                                                        -joystick3.customRight().getX())))),
                                () -> Constants.getCoralstate() == Constants.coralstate.Holding),
                            // choose pos2/3 when elevatorpos == 2
                            () -> elevator1.elevatorpos() == 2),
                        // choose pos4 when elevatorpos == 4
                        () -> elevator1.elevatorpos() == 4),
                    // choose pos1 when elevatorpos == 1
                    () -> elevator1.elevatorpos() == 1),
                // outer: only do this in ALGEA mode
                () -> Constants.getRobotState() != Constants.RobotState.ALGEA))
        .whileFalse(
            new SequentialCommandGroup(
                elevator1.Motionmagictoggle(0), new AutonElevatorcmd(elevator1, 0, false)));

    // ─── Right Bumper ──────────────────────────────────────────────────────────────
    joystick3
        .rightBumper()
        .whileTrue(
            new ConditionalCommand(
                // NOT ALGEA → go to l2right
                l2right,
                // ALGEA MODE → pos1, then pos4, then pos2/3
                new ConditionalCommand(
                    // pos1 → pivot to –7
                    new ParallelCommandGroup(
                        drivetrain.applyRequest(
                            () ->
                                drive
                                    .withVelocityX(MaxSpeed.times(-joystick3.customLeft().getY()))
                                    .withVelocityY(MaxSpeed.times(-joystick3.customLeft().getX()))
                                    .withRotationalRate(
                                        Constants.MaxAngularRate.times(
                                            -joystick3.customRight().getX()))),
                        new l3algae(algea, -0.7, 5, elevator1, -0.4, 0)),
                    // else → pos4
                    new ConditionalCommand(
                        // pos4 → pivot to 25.444…
                        new ParallelCommandGroup(
                            drivetrain.applyRequest(
                                () ->
                                    drive
                                        .withVelocityX(
                                            MaxSpeed.times(-joystick3.customLeft().getY()))
                                        .withVelocityY(
                                            MaxSpeed.times(-joystick3.customLeft().getX()))
                                        .withRotationalRate(
                                            Constants.MaxAngularRate.times(
                                                -joystick3.customRight().getX()))),
                            new barge(elevator1, 25.44423828125, true, algea)),
                        // else → pos2/3 nested
                        new ConditionalCommand(
                            // pos2
                            new ConditionalCommand(
                                new ParallelCommandGroup(
                                    hyper1,
                                    drivetrain.applyRequest(
                                        () ->
                                            drive
                                                .withVelocityX(
                                                    MaxSpeed.times(-joystick3.customLeft().getY()))
                                                .withVelocityY(
                                                    MaxSpeed.times(-joystick3.customLeft().getX()))
                                                .withRotationalRate(
                                                    Constants.MaxAngularRate.times(
                                                        -joystick3.customRight().getX())))),
                                new ParallelCommandGroup(
                                    Positionl21,
                                    drivetrain.applyRequest(
                                        () ->
                                            drive
                                                .withVelocityX(
                                                    MaxSpeed.times(-joystick3.customLeft().getY()))
                                                .withVelocityY(
                                                    MaxSpeed.times(-joystick3.customLeft().getX()))
                                                .withRotationalRate(
                                                    Constants.MaxAngularRate.times(
                                                        -joystick3.customRight().getX())))),
                                () -> Constants.getCoralstate() == Constants.coralstate.Holding),
                            // pos3
                            new ConditionalCommand(
                                new ParallelCommandGroup(
                                    hyperl31,
                                    drivetrain.applyRequest(
                                        () ->
                                            drive
                                                .withVelocityX(
                                                    MaxSpeed.times(-joystick3.customLeft().getY()))
                                                .withVelocityY(
                                                    MaxSpeed.times(-joystick3.customLeft().getX()))
                                                .withRotationalRate(
                                                    Constants.MaxAngularRate.times(
                                                        -joystick3.customRight().getX())))),
                                new ParallelCommandGroup(
                                    Positionl31,
                                    drivetrain.applyRequest(
                                        () ->
                                            drive
                                                .withVelocityX(
                                                    MaxSpeed.times(-joystick3.customLeft().getY()))
                                                .withVelocityY(
                                                    MaxSpeed.times(-joystick3.customLeft().getX()))
                                                .withRotationalRate(
                                                    Constants.MaxAngularRate.times(
                                                        -joystick3.customRight().getX())))),
                                () -> Constants.getCoralstate() == Constants.coralstate.Holding),
                            () -> elevator1.elevatorpos() == 2),
                        () -> elevator1.elevatorpos() == 4),
                    () -> elevator1.elevatorpos() == 1),
                () -> Constants.getRobotState() != Constants.RobotState.ALGEA))
        .whileFalse(
            new SequentialCommandGroup(
                elevator1.Motionmagictoggle(0), new AutonElevatorcmd(elevator1, 0, false)));

    joystick3
        .back()
        .whileTrue(new Elevatorcmd(elevator1, true))
        .whileFalse(
            new SequentialCommandGroup(
                elevator1.Motionmagictoggle(0), new AutonElevatorcmd(elevator1, 0, false)));
    // Elevator pos up
    joystick3.leftStick().whileTrue(elevator1.runOnce(() -> elevator1.elevatorup()));
    // Toggle Vision
    joystick3.b().whileTrue(new Lynkalighnmentleft(drivetrain, true, true, 1.5, 0));

    // X wheels
    joystick3.x().whileTrue(drivetrain.brake());

    // joystick3.x().whileTrue();
    // Trough
    // joystick3
    //     .b()
    //     .whileTrue(new AutonElevatorcmd(elevator1, 1, true))
    //     .whileFalse(
    //         new SequentialCommandGroup(
    //             elevator1.Motionmagictoggle(0), new AutonElevatorcmd(elevator1, 0, false)));
    // Reset Gyro
    // joystick3.b().whileTrue(drivetrain.runOnce(() -> drivetrain.resetgyro()));
    // Elevator position down
    joystick3.rightStick().whileTrue(elevator1.runOnce(() -> elevator1.elevatordown()));
    // Toggle Robot Sate
    joystick3.start().whileTrue(elevator1.runOnce(() -> elevator1.togglesetpoint()));

    joystick3
        .y()
        .whileTrue(new ParallelCommandGroup(climb.cmdspeed(1)))
        .whileFalse(climb.cmdspeed(0));

    joystick3
        .a()
        .whileTrue(new ParallelCommandGroup(climb.cmdspeed(-1)))
        .whileFalse(climb.cmdspeed(0));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  @Override
  public void robotPeriodic() {
    Threads.setCurrentThreadPriority(true, 99);
    CommandScheduler.getInstance().run();
    Threads.setCurrentThreadPriority(false, 10);
    SmartDashboard.putString("State", Constants.getRobotState().name());
    SmartDashboard.putString("Elevator Position", Constants.getElevatorState().name());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
