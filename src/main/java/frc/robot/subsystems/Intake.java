// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final TalonFX intakeMotor = new TalonFX(11, "rio");
  private final CANrange coolSensor = new CANrange(3, "rio");
  // private final TalonFX rightMotor = new TalonFX(10);

  public Intake() {}

  public Command intakeUntilSensor() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.

    return runOnce(
            () -> {
              intakeMotor.set(0.2);
            })
        .until(() -> coolSensor.getIsDetected().refresh().getValue())
        .andThen(Commands.waitSeconds(2))
        .andThen(() -> intakeMotor.stopMotor());
  }

  public Command start() {
    return runOnce(() -> intakeMotor.set(1.0));
  }

  public Command stop() {
    return runOnce(() -> intakeMotor.stopMotor());
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
