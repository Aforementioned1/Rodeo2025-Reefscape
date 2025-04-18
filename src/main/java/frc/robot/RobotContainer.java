// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.limelightPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.NamedCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.manipulator.Intake;
import frc.robot.subsystems.manipulator.IntakeIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.AllianceFlipUtil;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private boolean hasRunAutoOnceBefore = false;
  private final Drive drive;
  private final RobotState robotState = RobotState.getInstance();
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(10);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(10);
  private final SlewRateLimiter angularLimiter = new SlewRateLimiter(10);
  private final Elevator elevator;
  private final Intake intake;
  private LoggedNetworkNumber elevatorRef = new LoggedNetworkNumber("ElevatorReference", 1);
  private LoggedNetworkNumber intakeSpeedCurrent =
      new LoggedNetworkNumber("IntakeSpeedCurrent", 100);
  private LoggedNetworkNumber outtakeSpeedCurrent =
      new LoggedNetworkNumber("OuttakeSpeedCurrent", 100);
  private LoggedNetworkNumber outtakeSpeedL1Current =
      new LoggedNetworkNumber("OuttakeSpeedL1Current", 100);
  private LoggedNetworkNumber outtakeSpeedL4Current =
      new LoggedNetworkNumber("OuttakeSpeedL4Current", (100));
  private LoggedNetworkNumber intakeSpeed = new LoggedNetworkNumber("IntakeSpeed", 0.2);
  private LoggedNetworkNumber outtakeSpeed = new LoggedNetworkNumber("OuttakeSpeed", 0.45);
  private LoggedNetworkNumber outtakeSpeedL4 = new LoggedNetworkNumber("OuttakeSpeedL4", (0.25));
  private final Vision vision;

  @AutoLogOutput private int autoScoreBranch = 0;
  @AutoLogOutput private FieldConstants.ReefLevel autoScoreReefLevel = FieldConstants.ReefLevel.L4;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController buttonBoard = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<PathPlannerAuto> autoChooser;
  private final LoggedNetworkNumber l1Offset =
      new LoggedNetworkNumber("SmartDashboard/ElevatorOffsets/L1", 0.0);
  private final LoggedNetworkNumber l2Offset =
      new LoggedNetworkNumber("SmartDashboard/ElevatorOffsets/L2", 0.0);
  private final LoggedNetworkNumber l3Offset =
      new LoggedNetworkNumber("SmartDashboard/ElevatorOffsets/L3", 0.0);
  private final LoggedNetworkNumber l4Offset =
      new LoggedNetworkNumber("SmartDashboard/ElevatorOffsets/L4", 1.5);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    intake = new Intake(new IntakeIOTalonFX());
    elevator = new Elevator(new ElevatorIOTalonFX());
    vision =
        new Vision(
            RobotState.getInstance()::addVisionObservation,
            new VisionIOLimelight(limelightPose, camera0Name, robotState::getRotation));
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    new NamedCommands(drive, elevator, intake, vision);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", buildAutoChooser(""));

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization",
        new PathPlannerAuto(DriveCommands.wheelRadiusCharacterization(drive)));
    autoChooser.addOption(
        "Drive Simple FF Characterization",
        new PathPlannerAuto(DriveCommands.feedforwardCharacterization(drive)));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        new PathPlannerAuto(drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        new PathPlannerAuto(drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)",
        new PathPlannerAuto(drive.sysIdDynamic(SysIdRoutine.Direction.kForward)));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)",
        new PathPlannerAuto(drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)));
    // autoChooser.addOption(
    //     "1 Meter Test", );

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () ->
                -xLimiter.calculate(
                    Math.copySign(Math.pow(controller.getLeftY(), 2), controller.getLeftY())),
            () ->
                -yLimiter.calculate(
                    Math.copySign(Math.pow(controller.getLeftX(), 2), controller.getLeftX())),
            () ->
                -angularLimiter.calculate(
                    Math.copySign(Math.pow(controller.getRightX(), 2), controller.getRightX()))));

    // Lock to 0° when A button is held
    // controller
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .povDown()
        .onTrue(
            Commands.runOnce(
                    () ->
                        robotState.resetPose(
                            new Pose2d(
                                robotState.getEstimatedPose().getTranslation(), new Rotation2d(0))),
                    drive)
                .ignoringDisable(true));

    controller.leftTrigger().onTrue(intake.intakeUntilSensor(intakeSpeed::get));
    controller
        .rightTrigger()
        .onTrue(
            Commands.either(
                Commands.either(
                    intake.runTorqueCurrent(outtakeSpeedCurrent::get),
                    intake.runTorqueCurrent(outtakeSpeedL1Current::get),
                    () -> autoScoreReefLevel != FieldConstants.ReefLevel.L1),
                intake.runTorqueCurrent(outtakeSpeedL4Current::get),
                () -> autoScoreReefLevel != FieldConstants.ReefLevel.L4));
    // Commands.either(
    //     intake.runDutyCycle(outtakeSpeed::get),
    //     intake.runDutyCycle(outtakeSpeedL4::get),
    //     () -> autoScoreReefLevel != FieldConstants.ReefLevel.L4));
    controller.rightTrigger().onFalse(intake.stop());
    controller.a().onTrue(intake.runTorqueCurrent(() -> -intakeSpeedCurrent.get()));
    // controller.a().onTrue(intake.runDutyCycle(() -> -intakeSpeed.get()));
    controller.a().onFalse(intake.stop());

    // controller.a().onTrue();
    // controller.x().onTrue(intake.outtakeUntilSensor(intakeSpeed::get));
    // controller.leftTrigger().onTrue(intake.runNegativeDutyCycle(intakeSpeed::get));
    // controller.leftTrigger().onFalse(intake.stop());

    // temporary controller bindings until button board is finished
    // controller.leftBumper().onTrue(elevator.setPosition(elevatorRef::get));
    // controller
    //     .a()
    //     .onTrue(
    //         Commands.runOnce(() -> elevatorRef.set(1))
    //             .andThen(elevator.setPosition(elevatorRef::get)));
    // controller
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(() -> elevatorRef.set(23))
    //             .andThen(elevator.setPosition(elevatorRef::get)));
    // controller
    //     .x()
    //     .onTrue(
    //         Commands.runOnce(() -> elevatorRef.set(38))

    //             .andThen(elevator.setPosition(elevatorRef::get)));
    // controller
    //     .y()
    //     .onTrue(
    //         Commands.runOnce(() -> elevatorRef.set(62.5))
    //             .andThen(elevator.setPosition(elevatorRef::get)));

    // elevator buttons
    // set elevator to reference
    controller
        .y()
        .onTrue(
            elevator.setPosition(
                () ->
                    switch (autoScoreReefLevel) {
                          case L1 -> l1Offset.get();
                          case L2 -> l2Offset.get();
                          case L3 -> l3Offset.get();
                          case L4 -> l4Offset.get();
                        }
                        + autoScoreReefLevel.height));

    for (int i = 1; i < 13; i++) {
      int finalI = i - 1;
      buttonBoard
          .button(i)
          .onTrue(Commands.runOnce(() -> autoScoreBranch = finalI >= 4 ? finalI - 4 : finalI + 8));
    }
    // L0 / intake
    controller
        .x()
        .onTrue(
            elevator.setPosition(
                () -> 1)); /*.andThen(Commands.runOnce(() -> elevatorRef.set(1))) */
    // L1: 14.5
    // L2: 19.5
    // L3: 35.5
    // L4: 60.5
    // L1 - needs more testing - may not be possible - 18 right now
    buttonBoard
        .axisGreaterThan(0, 0.9)
        .onTrue(Commands.runOnce(() -> autoScoreReefLevel = FieldConstants.ReefLevel.L1));
    // L2 -
    buttonBoard
        .axisLessThan(1, -0.9)
        .onTrue(Commands.runOnce(() -> autoScoreReefLevel = FieldConstants.ReefLevel.L2));
    // L3
    buttonBoard
        .axisLessThan(0, -0.9)
        .onTrue(Commands.runOnce(() -> autoScoreReefLevel = FieldConstants.ReefLevel.L3));
    // L4
    buttonBoard
        .axisGreaterThan(1, 0.9)
        .onTrue(Commands.runOnce(() -> autoScoreReefLevel = FieldConstants.ReefLevel.L4));

    controller
        .leftBumper()
        .whileTrue(
            Commands.either(
                AutoScore.getAutoDrive( // if have a game piece, auto align
                    drive,
                    () ->
                        Optional.of(
                            new FieldConstants.CoralObjective(autoScoreBranch, autoScoreReefLevel)),
                    () -> autoScoreReefLevel,
                    () -> xLimiter.calculate(-controller.getLeftY()),
                    () -> yLimiter.calculate(-controller.getLeftX()),
                    () -> angularLimiter.calculate(-controller.getRightX())),
                AutoScore.getAutoDrive(
                    drive,
                    () -> {
                      if (FieldConstants.CoralStation.leftRegion.inRegion(
                          robotState.getEstimatedPose().getTranslation()))
                        return AllianceFlipUtil.apply(FieldConstants.CoralStation.leftCenterFace)
                            .plus(new Transform2d(0.5, 0, Rotation2d.kZero));
                      else if (FieldConstants.CoralStation.rightRegion.inRegion(
                          robotState.getEstimatedPose().getTranslation()))
                        return AllianceFlipUtil.apply(FieldConstants.CoralStation.rightCenterFace)
                            .plus(new Transform2d(0.5, 0, Rotation2d.kZero));
                      else return robotState.getEstimatedPose();
                    },
                    () -> xLimiter.calculate(-controller.getLeftY()),
                    () -> yLimiter.calculate(-controller.getLeftX()),
                    () -> angularLimiter.calculate(-controller.getRightX())),
                intake.haveAGamePiece()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  //  public void teleopInit() {
  //    DriveToPose.driveMaxAcceleration.
  //  }
  public Command getAutonomousCommand() {
    // return AutoBuilder.buildAuto("P5 first part").andThen(AutoBuilder.buildAuto("P5 second
    // part"));
    // return AutoBuilder.buildAuto("P2 first part").andThen(AutoBuilder.buildAuto("P2 second
    // part"));
    PathPlannerAuto auto = autoChooser.get();

    if (hasRunAutoOnceBefore) auto = new PathPlannerAuto(autoChooser.get().getName());

    hasRunAutoOnceBefore = true;
    return vision
        .seedPoseBeforeAuto(AllianceFlipUtil.apply(auto.getStartingPose()), Meters.of(1))
        .andThen(auto)
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
  }

  public SendableChooser<PathPlannerAuto> buildAutoChooser(String defaultAutoName) {
    SendableChooser<PathPlannerAuto> chooser = new SendableChooser<>();
    List<String> autoNames = AutoBuilder.getAllAutoNames();

    PathPlannerAuto defaultOption = null;

    for (String autoName : autoNames) {
      PathPlannerAuto auto = new PathPlannerAuto(autoName);

      if (!defaultAutoName.isEmpty() && defaultAutoName.equals(autoName)) {
        defaultOption = auto;
      } else {
        chooser.addOption(autoName, auto);
      }
    }

    if (defaultOption == null) {
      chooser.setDefaultOption("None", new PathPlannerAuto(Commands.none()));
    } else {
      chooser.setDefaultOption(defaultOption.getName(), defaultOption);
      chooser.addOption("None", new PathPlannerAuto(Commands.none()));
    }

    return chooser;
  }
}
