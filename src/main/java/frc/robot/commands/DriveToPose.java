// Copyright (c) 2025 FRC 3630
// https://github.com/Stampede3630
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.TimeDifferentiation;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {
  public static final LoggedTunableNumber drivekP = new LoggedTunableNumber("DriveToPose/DrivekP");
  public static final LoggedTunableNumber drivekD = new LoggedTunableNumber("DriveToPose/DrivekD");
  public static final LoggedTunableNumber thetakP = new LoggedTunableNumber("DriveToPose/ThetakP");
  public static final LoggedTunableNumber thetakD = new LoggedTunableNumber("DriveToPose/ThetakD");
  public static final LoggedTunableNumber driveMaxVelocity =
      new LoggedTunableNumber("DriveToPose/DriveMaxVelocity");
  public static final LoggedTunableNumber driveMaxVelocityTeleop =
      new LoggedTunableNumber("DriveToPose/DriveMaxVelocity");
  public static final LoggedTunableNumber driveMaxVelocitySlow =
      new LoggedTunableNumber("DriveToPose/DriveMaxVelocitySlow");
  public static final LoggedTunableNumber driveMaxAcceleration =
      new LoggedTunableNumber("DriveToPose/DriveMaxAcceleration");
  public static final LoggedTunableNumber driveMaxAccelerationTeleop =
      new LoggedTunableNumber("DriveToPose/DriveMaxAcceleration");
  public static final LoggedTunableNumber thetaMaxVelocity =
      new LoggedTunableNumber("DriveToPose/ThetaMaxVelocity");
  public static final LoggedTunableNumber thetaMaxAcceleration =
      new LoggedTunableNumber("DriveToPose/ThetaMaxAcceleration");
  public static final LoggedTunableNumber driveTolerance =
      new LoggedTunableNumber("DriveToPose/DriveTolerance");
  public static final LoggedTunableNumber thetaTolerance =
      new LoggedTunableNumber("DriveToPose/ThetaTolerance");
  public static final LoggedTunableNumber ffMinRadius =
      new LoggedTunableNumber("DriveToPose/FFMinRadius");
  public static final LoggedTunableNumber ffMaxRadius =
      new LoggedTunableNumber("DriveToPose/FFMaxRadius");

  static {
    drivekP.initDefault(1.4);
    drivekD.initDefault(0.14);
    thetakP.initDefault(4.0);
    thetakD.initDefault(0.0);
    driveMaxVelocity.initDefault(5); // auto
    driveMaxAcceleration.initDefault(6); // auto
    driveMaxVelocityTeleop.initDefault(5); // teleop
    driveMaxAccelerationTeleop.initDefault(6); // teleop
    thetaMaxVelocity.initDefault(Units.degreesToRadians(360.0));
    thetaMaxAcceleration.initDefault(8.0);
    driveTolerance.initDefault(0.03);
    thetaTolerance.initDefault(Units.degreesToRadians(3.0));
    ffMinRadius.initDefault(0.05);
    ffMaxRadius.initDefault(0.1);
  }

  private final Drive drive;
  private final Supplier<Pose2d> target;

  private final ProfiledPIDController driveController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);
  private final TimeDifferentiation driveErrorAbsDt =
      new TimeDifferentiation().withFilter(LinearFilter.singlePoleIIR(0.1, 0.02));
  private final TimeDifferentiation thetaErrorAbsDt =
      new TimeDifferentiation().withFilter(LinearFilter.singlePoleIIR(0.1, 0.02));
  private final LinearFilter driveErrorAbsFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
  private final LinearFilter thetaErrorAbsFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
  private Translation2d lastSetpointTranslation = new Translation2d();
  private double driveErrorAbs = 0.0;
  private double thetaErrorAbs = 0.0;
  @Getter private boolean running = false;
  private Supplier<Pose2d> robot;

  private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
  private DoubleSupplier omegaFF = () -> 0.0;

  public DriveToPose(Drive drive, Supplier<Pose2d> target) {
    this.drive = drive;
    this.target = target;

    // Enable continuous input for theta controller
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive);
  }

  public DriveToPose(Drive drive, Supplier<Pose2d> target, Supplier<Pose2d> robot) {
    this(drive, target);
    this.robot = robot;
  }

  public DriveToPose(
      Drive drive,
      Supplier<Pose2d> target,
      Supplier<Pose2d> robot,
      Supplier<Translation2d> linearFF,
      DoubleSupplier omegaFF) {
    this(drive, target, robot);
    this.linearFF = linearFF;
    this.omegaFF = omegaFF;
  }

  @Override
  public void initialize() {
    Pose2d currentPose = robot.get();
    ChassisSpeeds fieldVelocity =
        ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), currentPose.getRotation());
    Translation2d linearFieldVelocity =
        new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
    driveController.reset(
        currentPose.getTranslation().getDistance(target.get().getTranslation()),
        Math.min(
            0.0,
            -linearFieldVelocity
                .rotateBy(
                    target
                        .get()
                        .getTranslation()
                        .minus(currentPose.getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    thetaController.reset(
        currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
    lastSetpointTranslation = currentPose.getTranslation();
  }

  @Override
  public void execute() {
    running = true;

    // Update from tunable numbers
    if (driveMaxVelocity.hasChanged(hashCode())
        || driveMaxVelocitySlow.hasChanged(hashCode())
        || driveMaxAcceleration.hasChanged(hashCode())
        || driveTolerance.hasChanged(hashCode())
        || thetaMaxVelocity.hasChanged(hashCode())
        || thetaMaxAcceleration.hasChanged(hashCode())
        || thetaTolerance.hasChanged(hashCode())
        || drivekP.hasChanged(hashCode())
        || drivekD.hasChanged(hashCode())
        || thetakP.hasChanged(hashCode())
        || thetakD.hasChanged(hashCode())) {
      driveController.setP(drivekP.get());
      driveController.setD(drivekD.get());
      driveController.setConstraints(
          new TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get()));
      driveController.setTolerance(driveTolerance.get());
      thetaController.setP(thetakP.get());
      thetaController.setD(thetakD.get());
      thetaController.setConstraints(
          new TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get()));
      thetaController.setTolerance(thetaTolerance.get());
    }

    if (DriverStation.isTeleop() && driveMaxVelocityTeleop.hasChanged(hashCode())
        || driveMaxAccelerationTeleop.hasChanged(hashCode())) {
      driveController.setConstraints(
          new TrapezoidProfile.Constraints(
              driveMaxVelocityTeleop.get(), driveMaxAccelerationTeleop.get()));
    }

    // Get current pose and target pose
    Pose2d currentPose = robot.get();
    Pose2d targetPose = target.get();

    // Calculate drive speed
    double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    double ffScaler =
        MathUtil.clamp(
            (currentDistance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()),
            0.0,
            1.0);
    driveErrorAbs = currentDistance;
    driveErrorAbsFilter.calculate(driveErrorAbs);
    driveErrorAbsDt.calculate(driveErrorAbs);
    driveController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        driveController.getSetpoint().velocity);
    double driveVelocityScalar =
        driveController.getSetpoint().velocity * ffScaler
            + driveController.calculate(driveErrorAbs, 0.0);
    if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;
    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.toTransform2d(driveController.getSetpoint().position, 0.0))
            .getTranslation();

    // Calculate theta speed
    double thetaVelocity =
        thetaController.getSetpoint().velocity * ffScaler
            + thetaController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    thetaErrorAbsFilter.calculate(thetaErrorAbs);
    thetaErrorAbsDt.calculate(thetaErrorAbs);
    if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

    Translation2d driveVelocity =
        new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.toTransform2d(driveVelocityScalar, 0.0))
            .getTranslation();

    // Scale feedback velocities by input ff
    final double linearS = linearFF.get().getNorm() * 3.0;
    final double thetaS = Math.abs(omegaFF.getAsDouble()) * 3.0;
    driveVelocity =
        driveVelocity.interpolate(
            linearFF.get().times(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)), linearS);
    thetaVelocity =
        MathUtil.interpolate(
            thetaVelocity,
            omegaFF.getAsDouble()
                * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
                / Drive.DRIVE_BASE_RADIUS,
            thetaS);

    // Command speeds
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));

    // Log data
    Logger.recordOutput("DriveToPose/DistanceMeasured", currentDistance);
    Logger.recordOutput("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
    Logger.recordOutput("DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
    Logger.recordOutput("DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
    Logger.recordOutput(
        "DriveToPose/Setpoint",
        new Pose2d[] {
          new Pose2d(
              lastSetpointTranslation,
              Rotation2d.fromRadians(thetaController.getSetpoint().position))
        });
    Logger.recordOutput("DriveToPose/robotgiver", robot.get());
    Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {targetPose});
    Logger.recordOutput("DriveToPose/DriveErrorAbs", driveErrorAbs);
    Logger.recordOutput("DriveToPose/ThetaErrorAbs", thetaErrorAbs);
    Logger.recordOutput("DriveToPose/DriveErrorAbsFiltered", driveErrorAbsFilter.lastValue());
    Logger.recordOutput("DriveToPose/ThetaErrorAbsFiltered", thetaErrorAbsFilter.lastValue());
    Logger.recordOutput("DriveToPose/DriveErrorAbsdt", driveErrorAbsDt.getLastUnfilteredValue());
    Logger.recordOutput("DriveToPose/ThetaErrorAbsdt", thetaErrorAbsDt.getLastUnfilteredValue());
    Logger.recordOutput("DriveToPose/DriveErrorAbsdtFiltered", driveErrorAbsDt.getLastValue());
    Logger.recordOutput("DriveToPose/ThetaErrorAbsdtFiltered", thetaErrorAbsDt.getLastValue());
    Logger.recordOutput("DriveToPose/IsStuck", stuck());

    Logger.recordOutput("DriveToPose/AtGoal", atGoal());
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    running = false;
    // Clear logs
    Logger.recordOutput("DriveToPose/Setpoint", new Pose2d[] {});
    Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {});
  }

  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    return running && driveController.atGoal() && thetaController.atGoal();
  }

  /** Checks if the robot pose is within the allowed drive and theta tolerances. */
  public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return running
        && Math.abs(driveErrorAbs) < driveTolerance
        && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
  }

  public boolean stuck() {
    if (atGoal()) return false; // If at goal, not stuck
    // TODO tune these values
    // If the robot is not at goal and has steady-state error, but the error is not changing, it is
    // stuck
    return Math.abs(driveErrorAbsDt.getLastValue()) < 0.1 && driveErrorAbs > 0.08
        || Math.abs(thetaErrorAbsDt.getLastValue()) < 0.1 && thetaErrorAbs > 0.1;
  }
}
