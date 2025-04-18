package frc.robot.subsystems.manipulator;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  default void updateInputs(IntakeIOInputs inputs) {}

  default void stop() {}

  default boolean getSensor() {
    return false;
  }

  default boolean getSensorInAuto() {
    return false;
  }

  default void setTorqueCurrent(double amps) {}

  default void set(double dutyCycle) {}

  @AutoLog
  class IntakeIOInputs {
    public boolean connected = false;
    public double position = 0.0;
    public double velocity = 0.0;
    public double torqueCurrent = 0.0;
    public double voltage = 0.0;
    public double statorCurrent = 0.0;
    public double supplyCurrent = 0.0;
    public double temp = 0.0;
    public double sensorDistance = 0;
  }
}
