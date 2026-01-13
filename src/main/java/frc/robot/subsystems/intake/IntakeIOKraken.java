package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ConnectedMotorValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CurrentLimitConstants;
import frc.robot.Constants.IndexerConstants;
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

  public IntakeIOKraken(int top, int side) {
    m_motor = new TalonFX(top, Ports.kDriveCanivoreName);

    var currentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(CurrentLimitConstants.kIndexerDefaultSupplyLimit)
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(CurrentLimitConstants.kIndexerDefaultStatorLimit);

    var feedbackConfig =
        new FeedbackConfigs().withSensorToMechanismRatio(IndexerConstants.kGearRatio);

    var motorOutput = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);

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
        m_temperature
        );
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    BaseStatusSignal.refreshAll(
        m_connected,
        m_voltageSignal,
        m_velocity,
        m_supplyCurrent,
        m_statorCurrent,
        m_temperature
        );

    inputs.Connected = m_connected.getValue() != ConnectedMotorValue.Unknown;
    inputs.Voltage = m_voltageSignal.getValueAsDouble();
    inputs.Velocity = m_velocity.getValueAsDouble();
    inputs.SupplyCurrent = m_supplyCurrent.getValueAsDouble();
    inputs.StatorCurrent = m_statorCurrent.getValueAsDouble();
    inputs.Temperature = m_temperature.getValueAsDouble();
  }

  @Override
  public void setVoltage(double Voltage) {
    m_motor.setControl(m_voltage.withOutput(Voltage));
  }

  @Override
  public void setCurrentLimits(double supplyLimit) {
    m_motor
        .getConfigurator()
        .apply(m_config.CurrentLimits.withSupplyCurrentLimit(supplyLimit), 0.0);
  }
}