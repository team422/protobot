package frc.robot.subsystems.drive.intake;

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
  private TalonFX m_Motor;
  //private TalonFX m_sideMotor;
  private TalonFXConfiguration m_config;

  private VoltageOut m_Voltage = new VoltageOut(0).withEnableFOC(true);
  //private VoltageOut m_sideVoltage = new VoltageOut(0).withEnableFOC(true);

  private StatusSignal<ConnectedMotorValue> m_Connected;
  private StatusSignal<ConnectedMotorValue> m_sideConnected;
  private StatusSignal<Voltage> m_VoltageSignal;
  private StatusSignal<Voltage> m_sideVoltageSignal;
  private StatusSignal<AngularVelocity> m_Velocity;
  private StatusSignal<AngularVelocity> m_sideVelocity;
  private StatusSignal<Current> m_SupplyCurrent;
  private StatusSignal<Current> m_sideSupplyCurrent;
  private StatusSignal<Current> m_StatorCurrent;
  private StatusSignal<Current> m_sideStatorCurrent;
  private StatusSignal<Temperature> m_Temperature;
  private StatusSignal<Temperature> m_sideTemperature;

  public IntakeIOKraken(int top, int side) {
    m_Motor = new TalonFX(top, Ports.kDriveCanivoreName);
    //m_sideMotor = new TalonFX(side, Ports.kMainCanivoreName);

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

    //m_sideMotor.getConfigurator().apply(m_config);
    m_Motor.getConfigurator().apply(m_config);

    m_Connected = m_Motor.getConnectedMotor();
    //m_sideConnected = m_sideMotor.getConnectedMotor();

    m_VoltageSignal = m_Motor.getMotorVoltage();
    //m_sideVoltageSignal = m_sideMotor.getMotorVoltage();

    m_Velocity = m_Motor.getVelocity();
    //m_sideVelocity = m_sideMotor.getVelocity();

    m_SupplyCurrent = m_Motor.getSupplyCurrent();
    //m_sideSupplyCurrent = m_sideMotor.getSupplyCurrent();

    m_StatorCurrent = m_Motor.getStatorCurrent();
    //m_sideStatorCurrent = m_sideMotor.getStatorCurrent();

    m_Temperature = m_Motor.getDeviceTemp();
    //m_sideTemperature = m_sideMotor.getDeviceTemp();
    StatusSignal.setUpdateFrequencyForAll(
        50,
        m_Connected,
        //m_sideConnected,
        m_VoltageSignal,
        //m_sideVoltageSignal,
        m_Velocity,
        //m_sideVelocity,
        m_SupplyCurrent,
        //m_sideSupplyCurrent,
        m_StatorCurrent,
        //m_sideStatorCurrent,
        m_Temperature
        //m_sideTemperature
        );
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    BaseStatusSignal.refreshAll(
        m_Connected,
        //m_sideConnected,
        m_VoltageSignal,
        //m_sideVoltageSignal,
        m_Velocity,
        //m_sideVelocity,
        m_SupplyCurrent,
        //m_sideSupplyCurrent,
        m_StatorCurrent,
        //m_sideStatorCurrent,
        m_Temperature
        //m_sideTemperature
        );

    inputs.Connected = m_Connected.getValue() != ConnectedMotorValue.Unknown;
    //inputs.sideConnected = m_sideConnected.getValue() != ConnectedMotorValue.Unknown;

    inputs.Voltage = m_VoltageSignal.getValueAsDouble();
    //inputs.sideVoltage = m_sideVoltageSignal.getValueAsDouble();

    inputs.Velocity = m_Velocity.getValueAsDouble();
    //inputs.sideVelocity = m_sideVelocity.getValueAsDouble();

    inputs.SupplyCurrent = m_SupplyCurrent.getValueAsDouble();
    //inputs.sideSupplyCurrent = m_sideSupplyCurrent.getValueAsDouble();

    inputs.StatorCurrent = m_StatorCurrent.getValueAsDouble();
    //inputs.sideStatorCurrent = m_sideStatorCurrent.getValueAsDouble();

    inputs.Temperature = m_Temperature.getValueAsDouble();
    //inputs.bottomTemperature = m_sideTemperature.getValueAsDouble();
  }

  @Override
  public void setVoltage(double Voltage) {
    m_Motor.setControl(m_Voltage);
    //m_sideMotor.setControl(m_sideVoltage.withOutput(side));
  }
}