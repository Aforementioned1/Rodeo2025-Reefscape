// Copyright (c) 2025 FRC 3630
// https://github.com/Stampede3630
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants.CoralObjective;
import frc.robot.FieldConstants.Reef;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPose;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class AutoScore {
  public static final LoggedNetworkNumber xOffset =
      new LoggedNetworkNumber(
          "AutoScore/xOffsetInches", 19.35); // 19.35 before (MIGHT NOT ACTUALLY BE INCHES?)
  public static final LoggedNetworkNumber yOffset =
      new LoggedNetworkNumber(
          "AutoScore/yOffsetInches", -1.25); // -1.25 before - should be -1.25-0 ish
  public static final LoggedNetworkNumber yOffsetL4 =
      new LoggedNetworkNumber("AutoScore/yOffsetL4Inches", 0.75); // should be 0-2.5 ish
  public static final LoggedTunableNumber minDistanceReefClearAlgae =
      new LoggedTunableNumber("AutoScore/MinDistanceReefClearAlgae", Units.inchesToMeters(18.0));
  public static final LoggedTunableNumber minDistanceReefClear =
      new LoggedTunableNumber("AutoScore/MinDistanceReefClear", Units.inchesToMeters(12.0));
  public static final LoggedTunableNumber xOffsetL1 =
      new LoggedTunableNumber("AutoScore/xOffsetL1Meters", 0.6);
  public static final LoggedTunableNumber yOffsetL1 =
      new LoggedTunableNumber("AutoScore/yOffsetL1Inches", Units.inchesToMeters(0.0));
  public static final LoggedTunableNumber degreeOffsetL1 =
      new LoggedTunableNumber("AutoScore/degreeOffsetL1", 150.0);

  // Radius of regular hexagon is side length
  private static final double reefRadius = Reef.faceLength;
  private static final LoggedTunableNumber maxDistanceReefLineup =
      new LoggedTunableNumber("AutoScore/MaxDistanceReefLineup", 1.5);
  private static final LoggedTunableNumber distanceSuperstructureReady =
      new LoggedTunableNumber("AutoScore/DistanceSuperstructureReady", Units.inchesToMeters(72.0));
  private static final LoggedTunableNumber[] linearXToleranceEject = {
    new LoggedTunableNumber("AutoScore/LinearXToleranceEject/L1", 0.05),
    new LoggedTunableNumber("AutoScore/LinearXToleranceEject/L2", 0.15),
    new LoggedTunableNumber("AutoScore/LinearXToleranceEject/L3", 0.15),
    new LoggedTunableNumber("AutoScore/LinearXToleranceEject/L4", 0.02)
  };
  private static final LoggedTunableNumber[] linearYToleranceEject = {
    new LoggedTunableNumber("AutoScore/LinearYToleranceEject/L1", 0.05),
    new LoggedTunableNumber("AutoScore/LinearYToleranceEject/L2", 0.015),
    new LoggedTunableNumber("AutoScore/LinearYToleranceEject/L3", 0.015),
    new LoggedTunableNumber("AutoScore/LinearYToleranceEject/L4", 0.01)
  };
  private static final LoggedTunableNumber[] maxLinearVel = {
    new LoggedTunableNumber("AutoScore/MaxLinearVel/L1", 3),
    new LoggedTunableNumber("AutoScore/MaxLinearVel/L2", 3),
    new LoggedTunableNumber("AutoScore/MaxLinearVel/L3", 3),
    new LoggedTunableNumber("AutoScore/MaxLinearVel/L4", 3)
  };
  private static final LoggedTunableNumber[] maxAngularVel = {
    new LoggedTunableNumber("AutoScore/MaxAngularVel/L1", 3),
    new LoggedTunableNumber("AutoScore/MaxAngularVel/L2", 3),
    new LoggedTunableNumber("AutoScore/MaxAngularVel/L3", 3),
    new LoggedTunableNumber("AutoScore/MaxAngularVel/L4", 3)
  };
  private static final LoggedTunableNumber thetaToleranceEject =
      new LoggedTunableNumber("AutoScore/ThetaToleranceEject", 2.0);
  private static final LoggedTunableNumber minDistanceAim =
      new LoggedTunableNumber("AutoScore/MinDistanceAim", 0.2);
  private static final LoggedTunableNumber ejectTimeSeconds =
      new LoggedTunableNumber("AutoScore/EjectTimeSeconds", 0.5);

  private AutoScore() {}

  public static Command getAutoDriveBlocking(
      Drive drive,
      Supplier<Optional<CoralObjective>> coralObjective,
      Supplier<FieldConstants.ReefLevel> reefLevel) {
    Supplier<Pose2d> robot =
        () ->
            coralObjective
                .get()
                .map(AutoScore::getRobotPose)
                .orElseGet(() -> RobotState.getInstance().getEstimatedPose());
    DriveToPose driveToPose =
        new DriveToPose(
            drive,
            () ->
                coralObjective
                    .get()
                    .map(
                        objective -> {
                          // use L1 offsets if L1 is selected (MAY NOT WORK)
                          if (reefLevel.get() == FieldConstants.ReefLevel.L1) {
                            return getDriveTarget(
                                robot.get(),
                                AllianceFlipUtil.apply(getCoralScorePoseL1(objective)));
                          }

                          // use L4 y offset if L4 is selected
                          if (reefLevel.get() == FieldConstants.ReefLevel.L4) {
                            return getDriveTarget(
                                robot.get(),
                                AllianceFlipUtil.apply(getCoralScorePoseL4(objective)));
                          }

                          // default to using normal offsets
                          Pose2d goalPose = getCoralScorePose(objective);
                          return getDriveTarget(robot.get(), AllianceFlipUtil.apply(goalPose));
                          //                      return AllianceFlipUtil.apply(goalPose);
                        })
                    .orElseGet(() -> RobotState.getInstance().getEstimatedPose()),
            robot);
    return driveToPose.until(driveToPose::atGoal).withTimeout(10);
  }

  public static Command getAutoDrive(
      Drive drive,
      Supplier<Optional<CoralObjective>> coralObjective,
      Supplier<FieldConstants.ReefLevel> reefLevel,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega) {
    Supplier<Pose2d> robot =
        () ->
            coralObjective
                .get()
                .map(AutoScore::getRobotPose)
                .orElseGet(() -> RobotState.getInstance().getEstimatedPose());
    return new DriveToPose(
        drive,
        () ->
            coralObjective
                .get()
                .map(
                    objective -> {
                      // use L1 offsets if L1 is selected (MAY NOT WORK)
                      if (reefLevel.get() == FieldConstants.ReefLevel.L1) {
                        return getDriveTarget(
                            robot.get(), AllianceFlipUtil.apply(getCoralScorePoseL1(objective)));
                      }

                      // use L4 y offset if L4 is selected
                      if (reefLevel.get() == FieldConstants.ReefLevel.L4) {
                        return getDriveTarget(
                            robot.get(), AllianceFlipUtil.apply(getCoralScorePoseL4(objective)));
                      }
                      // default to using normal offsets
                      Pose2d goalPose = getCoralScorePose(objective);
                      return getDriveTarget(robot.get(), AllianceFlipUtil.apply(goalPose));
                      //                      return AllianceFlipUtil.apply(goalPose);
                    })
                .orElseGet(() -> RobotState.getInstance().getEstimatedPose()),
        robot,
        () ->
            DriveCommands.getLinearVelocityFromJoysticks(
                    driverX.getAsDouble(), driverY.getAsDouble())
                .times(AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0),
        () -> DriveCommands.getOmegaFromJoysticks(driverOmega.getAsDouble()));
  }

  public static Command getAutoDrive(
      Drive drive,
      Supplier<Pose2d> pose2dSupplier,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega) {
    Supplier<Pose2d> robot = () -> RobotState.getInstance().getEstimatedPose();
    return new DriveToPose(
        drive,
        pose2dSupplier,
        robot,
        () ->
            DriveCommands.getLinearVelocityFromJoysticks(
                    driverX.getAsDouble(), driverY.getAsDouble())
                .times(AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0),
        () -> DriveCommands.getOmegaFromJoysticks(driverOmega.getAsDouble()));
  }

  /** Get drive target. */
  public static Pose2d getDriveTarget(Pose2d robot, Pose2d goal) {
    var offset = robot.relativeTo(goal);
    double yDistance = Math.abs(offset.getY());
    double xDistance = Math.abs(offset.getX());
    double shiftXT =
        MathUtil.clamp(
            (yDistance / (Reef.faceLength * 2)) + ((xDistance - 0.3) / (Reef.faceLength * 3)),
            0.0,
            1.0);
    double shiftYT =
        MathUtil.clamp(yDistance <= 0.2 ? 0.0 : offset.getX() / Reef.faceLength, 0.0, 1.0);
    return goal.transformBy(
        GeomUtil.toTransform2d(
            -shiftXT * maxDistanceReefLineup.get(),
            Math.copySign(shiftYT * maxDistanceReefLineup.get() * 0.8, offset.getY())));
  }

  /** Get position of robot aligned with branch for selected objective. */
  public static Pose2d getCoralScorePose(CoralObjective coralObjective) {
    return getBranchPose(coralObjective)
        .transformBy(
            new Transform2d(Inches.of(xOffset.get()), Inches.of(yOffset.get()), Rotation2d.kZero))
        .plus(new Transform2d(0, 0, Rotation2d.k180deg));
  }

  // alignment is different for L4 due to lean of the elevator
  public static Pose2d getCoralScorePoseL4(CoralObjective coralObjective) {
    return getBranchPose(coralObjective)
        .transformBy(
            new Transform2d(Inches.of(xOffset.get()), Inches.of(yOffsetL4.get()), Rotation2d.kZero))
        .plus(new Transform2d(0, 0, Rotation2d.k180deg));
  }

  // L1 alignment - MAY NOT WORK
  private static Pose2d getCoralScorePoseL1(CoralObjective coralObjective) {
    // int face = coralObjective.branchId() / 2;
    // return Reef.centerFaces[face].transformBy(
    //     new Transform2d(
    //         Inches.of(xOffsetL1.get()),
    //         Inches.of(yOffsetL1.get() * (coralObjective.branchId() % 2 == 0 ? 1.0 : -1.0)),
    //         Rotation2d.fromDegrees(
    //             degreeOffsetL1.get() * (coralObjective.branchId() % 2 == 0 ? 1.0 : -1.0))));
    int face = coralObjective.branchId() / 2;
    return Reef.centerFaces[face].transformBy(
        new Transform2d(
            xOffsetL1.get(),
            yOffsetL1.get() * (coralObjective.branchId() % 2 == 0 ? 1.0 : 1.0),
            Rotation2d.fromDegrees(
                degreeOffsetL1.get() * (coralObjective.branchId() % 2 == 0 ? 1.0 : 1.0))));
  }

  public static boolean withinDistanceToReef(Pose2d robot, double distance) {
    final double distanceToReefCenter =
        AllianceFlipUtil.apply(robot).getTranslation().getDistance(Reef.center);
    Logger.recordOutput("AutoScore/DistanceToReefCenter", distanceToReefCenter);
    return distanceToReefCenter
        <= reefRadius
            + (TunerConstants.BackLeft.LocationX - TunerConstants.BackRight.LocationX) / 2.0
            + distance;
  }

  public static boolean outOfDistanceToReef(Pose2d robot, double distance) {
    final double distanceToReefCenter =
        AllianceFlipUtil.apply(robot).getTranslation().getDistance(Reef.center);
    Logger.recordOutput("AutoScore/DistanceToReefCenter", distanceToReefCenter);
    return distanceToReefCenter
        >= reefRadius
            + (TunerConstants.BackLeft.LocationX - TunerConstants.BackRight.LocationX) / 2.0
            + distance;
  }

  public static Pose2d getRobotPose(CoralObjective coralObjective) {
    return RobotState.getInstance()
        .getReefPose(coralObjective.branchId() / 2, getCoralScorePose(coralObjective));
  }

  public static Pose2d getBranchPose(CoralObjective objective) {
    return Reef.branchPositions2d.get(objective.branchId()).get(objective.reefLevel());
  }
}
