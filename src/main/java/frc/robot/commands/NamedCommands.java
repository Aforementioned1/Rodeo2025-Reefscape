// Copyright (c) 2025 FRC 3630
// https://github.com/Stampede3630
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.AutoScore;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.manipulator.Intake;
import frc.robot.subsystems.vision.Vision;
import java.util.HashMap;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import lombok.Getter;

public class NamedCommands {
  @Getter private final HashMap<String, Command> commands = new HashMap<String, Command>();
  private final Drive drive;
  private final Elevator elevator;
  private final Intake intake;
  private final Vision vision;

  public NamedCommands(Drive drive, Elevator elevator, Intake intake, Vision vision) {
    this.drive = drive;
    this.elevator = elevator;
    this.intake = intake;
    this.vision = vision;
    //    for (int i = 0; i < 12; i++) {
    //      commands.put(
    //          "scoreCoral" + i + "L4",
    //          getAutoScore(
    //              Optional.of(new FieldConstants.CoralObjective(i,
    // FieldConstants.ReefLevel.L4))));
    //    }
    for (int i = 65; i <= 76; i++) {
      int coralPlacement = 14 - (i / 65 + i % 65);
      if (coralPlacement > 11) {
        coralPlacement -= 12;
      }
      commands.put(
          "scoreCoral" + (char) i + "L4",
          getAutoScore(
              Optional.of(
                  new FieldConstants.CoralObjective(coralPlacement, FieldConstants.ReefLevel.L4))));
    }
    commands.put("intakeCoral", intake.intakeUntilSensor(() -> 0.2));
    commands.put("waitToGetCoral", Commands.waitUntil(intake.haveAGamePiece()));
    // commands.put("intakeCoral", elevator.intakeHeight().andThen(intake.autoIntake()));

    com.pathplanner.lib.auto.NamedCommands.registerCommands(commands);
  }

  private Command getAutoScore(Optional<FieldConstants.CoralObjective> objective) {
    DoubleSupplier elevHeight = () -> objective.get().reefLevel().height;
    return
    //        elevator.setPosition(elevHeight)
    //        .alongWith(
    AutoScore.getAutoDriveBlocking(
            drive,
            () -> objective,
            () ->
                objective
                    .map(FieldConstants.CoralObjective::reefLevel)
                    .orElse(FieldConstants.ReefLevel.L4))
        .withTimeout(Seconds.of(3) /* stop auto aligning after 5 seconds*/)
        .andThen(
            elevator.setPositionBlocking(
                elevHeight,
                Seconds.of(10000))) // maybe should be 10? (was the measurement originally in ms?)
        .andThen(Commands.waitSeconds(0.1))
        .andThen(intake.outtakeUntilSensor(() -> 0.3))
        .andThen(elevator.setPositionBlocking(() -> 1, Seconds.of(1000)));
  }
}
