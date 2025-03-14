// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private double setpoint = -1;

  public Elevator(ElevatorIO elevatorIO) {
    this.io = elevatorIO;
  }

  public Command setPosition(DoubleSupplier position) {
    return runOnce(
        () -> {
          setpoint = position.getAsDouble();
          io.runPosition(position.getAsDouble());
        });
  }

  public Command setPositionBlocking(DoubleSupplier position, Time timeout) {
    return setPosition(position).andThen(Commands.waitUntil(this::atGoal)).withTimeout(timeout);
  }

  public boolean atGoal() {
    return atGoal(0.5);
  }

  public boolean atGoal(double epsilon) {
    return Math.abs(inputs.position - setpoint) <= epsilon;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
