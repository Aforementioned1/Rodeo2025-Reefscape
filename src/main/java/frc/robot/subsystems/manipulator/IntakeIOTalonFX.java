// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX intakeMotor = new TalonFX(11, "rio");
  private final CANrange coolSensor = new CANrange(3, "rio");
  // private final TalonFX rightMotor = new TalonFX(10);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Temperature> temp;
  private final StatusSignal<Distance> sensorDistance;

  public IntakeIOTalonFX() {
    intakeMotor
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withMotorOutput(
                    new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)));

    position = intakeMotor.getPosition();
    velocity = intakeMotor.getVelocity();
    torqueCurrent = intakeMotor.getTorqueCurrent();
    statorCurrent = intakeMotor.getStatorCurrent();
    supplyCurrent = intakeMotor.getSupplyCurrent();
    voltage = intakeMotor.getMotorVoltage();
    temp = intakeMotor.getDeviceTemp();
    sensorDistance = coolSensor.getDistance();
  }

  // public Command intakeUntilSensor() {
  //   // Inline construction of command goes here.
  //   // Subsystem::RunOnce implicitly requires `this` subsystem.

  //   return runOnce(
  //           () -> {
  //             intakeMotor.set(0.2);
  //           })
  //       .until(() -> coolSensor.getIsDetected().refresh().getValue())
  //       .andThen(Commands.waitSeconds(2))
  //       .andThen(() -> intakeMotor.stopMotor());
  // }

  public void set(double dutyCycle) {
    intakeMotor.set(dutyCycle);
  }

  public boolean getSensor() {
    return sensorDistance.getValueAsDouble() < 0.1;
  }

  public void stop() {
    intakeMotor.stopMotor();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    boolean connected =
        BaseStatusSignal.refreshAll(
                position,
                velocity,
                torqueCurrent,
                statorCurrent,
                supplyCurrent,
                voltage,
                temp,
                sensorDistance)
            .isOK();

    inputs.connected = connected;
    inputs.position = position.getValueAsDouble();
    inputs.velocity = velocity.getValueAsDouble();
    inputs.torqueCurrent = torqueCurrent.getValueAsDouble();
    inputs.voltage = voltage.getValueAsDouble();
    inputs.statorCurrent = statorCurrent.getValueAsDouble();
    inputs.supplyCurrent = supplyCurrent.getValueAsDouble();
    inputs.temp = temp.getValueAsDouble();
    inputs.sensorDistance = coolSensor.getDistance().getValueAsDouble();
  }
}
