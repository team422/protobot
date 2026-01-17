package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ConnectedMotorValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CurrentLimitConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.Ports;

public class IntakeIOKraken implements IntakeIO {
  private TalonFX m_motor;

  private TalonFXConfiguration m_config;

  private VoltageOut m_voltage = new VoltageOut(0).withEnableFOC(true);

  private StatusSignal<ConnectedMotorValue> m_connected;
  private StatusSignal<Voltage> m_voltageSignal;
  private StatusSignal<AngularVelocity> m_velocity;
  private StatusSignal<Current> m_supplyCurrent;
  private StatusSignal<Current> m_statorCurrent;
  private StatusSignal<Temperature> m_temperature;

  public IntakeIOKraken(int port) {
    m_motor = new TalonFX(port, Ports.kDriveCanivoreName);

    var currentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(CurrentLimitConstants.kIntakeDefaultSupplyLimit)
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(CurrentLimitConstants.kIntakeDefaultStatorLimit);

    var feedbackConfig =
        new FeedbackConfigs().withSensorToMechanismRatio(IntakeConstants.kGearRatio);

    var motorOutput =
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive);

    m_config =
        new TalonFXConfiguration()
            .withCurrentLimits(currentLimits)
            .withFeedback(feedbackConfig)
            .withMotorOutput(motorOutput);

    m_motor.getConfigurator().apply(m_config);

    m_connected = m_motor.getConnectedMotor();
    m_voltageSignal = m_motor.getMotorVoltage();
    m_velocity = m_motor.getVelocity();
    m_supplyCurrent = m_motor.getSupplyCurrent();
    m_statorCurrent = m_motor.getStatorCurrent();
    m_temperature = m_motor.getDeviceTemp();

    StatusSignal.setUpdateFrequencyForAll(
        50,
        m_connected,
        m_voltageSignal,
        m_velocity,
        m_supplyCurrent,
        m_statorCurrent,
        m_temperature);
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    StatusSignal.refreshAll(
        m_connected, m_voltageSignal, m_velocity, m_supplyCurrent, m_statorCurrent, m_temperature);

    inputs.connected = m_connected.getValue() != ConnectedMotorValue.Unknown;
    inputs.voltage = m_voltageSignal.getValueAsDouble();
    inputs.velocityRPS = m_velocity.getValueAsDouble();
    inputs.supplyCurrent = m_supplyCurrent.getValueAsDouble();
    inputs.statorCurrent = m_statorCurrent.getValueAsDouble();
    inputs.temperature = m_temperature.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    m_motor.setControl(m_voltage.withOutput(volts));
  }

  @Override
  public void setCurrentLimits(double supplyLimit) {
    m_motor
        .getConfigurator()
        .apply(m_config.CurrentLimits.withSupplyCurrentLimit(supplyLimit), 0.0);
  }
}
