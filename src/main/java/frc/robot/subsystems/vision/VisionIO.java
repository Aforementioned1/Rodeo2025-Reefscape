// Copyright (c) 2025 FRC 3630
// https://github.com/Stampede3630
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotState;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  default void updateInputs(VisionIOInputs inputs) {}

  enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTONVISION
  }

  @AutoLog
  class VisionIOInputs {
    public boolean connected = false;
    public TargetObservation latestTargetObservation =
        new TargetObservation(new Rotation2d(), new Rotation2d());
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
    public RobotState.TxTyObservation[] txTyObservations = new RobotState.TxTyObservation[0];
  }

  /** Represents the angle to a simple target, not used for pose estimation. */
  record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  /** Represents a robot pose sample used for pose estimation. */
  record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {}
}
