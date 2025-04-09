package frc.robot.subsystems.manipulator;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO intakeIO) {
    this.io = intakeIO;
  }

  public Command runDutyCycle(DoubleSupplier dutyCycle) {
    return runOnce(() -> io.set(dutyCycle.getAsDouble()));
  }

  public Command runNegativeDutyCycle(DoubleSupplier dutyCycle) {
    return runOnce(() -> io.set(-1 * dutyCycle.getAsDouble()));
  }

  //   public Command intakeUntilSensor(DoubleSupplier dutyCycle) {
  //     return runOnce(() -> io.set(dutyCycle.getAsDouble())).until(() -> io.getSensor());
  //   }

  public Command stop() {
    return runOnce(io::stop);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Manipulator", inputs);
  }

  public Command intakeUntilSensor(DoubleSupplier dutyCycle) {
    return startEnd(() -> io.set(dutyCycle.getAsDouble()), () -> {})
        .until(io::getSensor)
        .andThen(io::stop);
  }

  public Command outtakeUntilSensor(DoubleSupplier dutyCycle) {
    Debouncer debouncer = new Debouncer(0.1);
    return startEnd(() -> io.set(dutyCycle.getAsDouble()), () -> {})
        .until(() -> debouncer.calculate(!io.getSensorInAuto()))
        .andThen(io::stop);
  }

  public double getDistance() {
    return inputs.sensorDistance;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public BooleanSupplier haveAGamePiece() {
    return () -> io.getSensor();
  }
}
