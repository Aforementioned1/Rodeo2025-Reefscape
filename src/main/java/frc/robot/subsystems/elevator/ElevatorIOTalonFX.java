package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX leader;

  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Double> reference;
  private final StatusSignal<Current> leaderTorqueCurrent;
  private final StatusSignal<Current> leaderStatorCurrent;
  private final StatusSignal<Current> leaderSupplyCurrent;
  private final StatusSignal<Voltage> leaderVoltage;
  private final StatusSignal<Temperature> leaderTemp;
  private final StatusSignal<Current> followerTorqueCurrent;
  private final StatusSignal<Current> followerStatorCurrent;
  private final StatusSignal<Current> followerSupplyCurrent;
  private final StatusSignal<Voltage> followerVoltage;
  private final StatusSignal<Temperature> followerTemp;

  private final TalonFX follower;
  private final Debouncer leaderConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer followerConnectedDebouncer = new Debouncer(0.5);

  private double positionSetpoint = 0;
  private final MotionMagicExpoVoltage positionRequest =
      new MotionMagicExpoVoltage(positionSetpoint).withSlot(0).withEnableFOC(true);
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  public ElevatorIOTalonFX() {
    leader = new TalonFX(50, "CANIVORE");
    follower = new TalonFX(52, "CANIVORE");

    config
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake))
        .withSlot0(
            new Slot0Configs()
                .withKP(1)
                .withKG(0.29)
                .withKS(0.09)
                .withKV(.125)
                .withKA(0)
                .withKI(0)
                .withGravityType(GravityTypeValue.Elevator_Static))
        .withFeedback(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(1.09435)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor))
        .withSoftwareLimitSwitch(
            new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(64.5)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(0))
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicAcceleration(0)
                .withMotionMagicCruiseVelocity(0)
                .withMotionMagicExpo_kA(0.05) // 0.06 old
                .withMotionMagicExpo_kV(0.07)); // 0.08 old
    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);

    leader.setPosition(0);
    follower.setPosition(0);

    position = leader.getPosition();
    velocity = leader.getVelocity();
    reference = leader.getClosedLoopReference();
    leaderTorqueCurrent = leader.getTorqueCurrent();
    leaderStatorCurrent = leader.getStatorCurrent();
    leaderSupplyCurrent = leader.getSupplyCurrent();
    leaderVoltage = leader.getMotorVoltage();
    leaderTemp = leader.getDeviceTemp();
    followerTorqueCurrent = follower.getTorqueCurrent();
    followerStatorCurrent = follower.getStatorCurrent();
    followerSupplyCurrent = follower.getSupplyCurrent();
    followerVoltage = follower.getMotorVoltage();
    followerTemp = follower.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(10, leaderTemp, followerTemp);
    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        position,
        velocity,
        reference,
        leaderTorqueCurrent,
        leaderStatorCurrent,
        leaderSupplyCurrent);

    ParentDevice.optimizeBusUtilizationForAll(leader);

    follower.setControl(new Follower(leader.getDeviceID(), false));
  }

  @Override
  public void runPosition(double position) {
    positionSetpoint = position;
    leader.setControl(positionRequest.withPosition(positionSetpoint));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    boolean connected =
        BaseStatusSignal.refreshAll(
                position,
                velocity,
                reference,
                leaderTorqueCurrent,
                leaderStatorCurrent,
                leaderSupplyCurrent,
                leaderVoltage,
                leaderTemp)
            .isOK();
    boolean followerConnected =
        BaseStatusSignal.refreshAll(
                followerVoltage,
                followerTorqueCurrent,
                followerSupplyCurrent,
                followerStatorCurrent,
                followerTemp)
            .isOK();

    inputs.leaderConnected = leaderConnectedDebouncer.calculate(connected);
    inputs.followerConnected = followerConnectedDebouncer.calculate(followerConnected);
    inputs.position = position.getValueAsDouble();
    inputs.velocity = velocity.getValueAsDouble();
    inputs.reference = reference.getValueAsDouble();
    inputs.leaderVoltage = leaderVoltage.getValueAsDouble();
    inputs.leaderTorqueCurrent = leaderTorqueCurrent.getValueAsDouble();
    inputs.leaderStatorCurrent = leaderStatorCurrent.getValueAsDouble();
    inputs.leaderSupplyCurrent = leaderSupplyCurrent.getValueAsDouble();
    inputs.leaderTemp = leaderTemp.getValueAsDouble();
    inputs.followerVoltage = followerVoltage.getValueAsDouble();
    inputs.followerTorqueCurrent = followerTorqueCurrent.getValueAsDouble();
    inputs.followerStatorCurrent = followerStatorCurrent.getValueAsDouble();
    inputs.followerSupplyCurrent = followerSupplyCurrent.getValueAsDouble();
    inputs.followerTemp = followerTemp.getValueAsDouble();
    inputs.appliedPosition = positionRequest.Position;
  }

  @Override
  public void runVolts(double volts) {
    leader.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  @Override
  public boolean setCoastMode(boolean enabled) {
    return leader.setNeutralMode(enabled ? NeutralModeValue.Coast : NeutralModeValue.Brake).isOK()
        & follower.setNeutralMode(enabled ? NeutralModeValue.Coast : NeutralModeValue.Brake).isOK();
  }

  @Override
  public boolean seedPosition(double newPosition) {
    return leader.setPosition(newPosition).isOK();
  }
}
