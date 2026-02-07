package frc.robot.subsystems.shoryaInstruction;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ConnectedMotorValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.CurrentLimitConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.ShoryaConstants;
import frc.robot.util.CtreBaseRefreshManager;

public class shoryaInstructionIOKraken implements shoryaInstructionIO {
    private TalonFX m_motor;

    private StatusSignal<Angle> m_motorPosition;
    private StatusSignal<AngularVelocity> m_motorVelocity;
    private StatusSignal<Voltage> m_motorVoltage;
    private StatusSignal<ConnectedMotorValue> m_connectedMotor;
    private StatusSignal<Current> m_motorCurrent;
    private StatusSignal<Current> m_motorStatorCurrent;
    private StatusSignal<Temperature> m_motorTemperature;
    private Rotation2d m_desiredAngle;

    private final TalonFXConfiguration m_config;

    private PositionVoltage m_positionControl = 
        new PositionVoltage(0.0).withSlot(0).withEnableFOC(true);
    
    public shoryaInstructionIOKraken(int port) {
        m_motor = new TalonFX(port, Ports.kMainCanivoreName);
    m_desiredAngle = new Rotation2d();

    var currentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(CurrentLimitConstants.kshoryaInstructionDefaultSupplyLimit)
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(CurrentLimitConstants.kshoryaInstructionDefaultStatorLimit);

    var motorOutput =
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive);

    var feedback =
        new FeedbackConfigs().withSensorToMechanismRatio(ShoryaConstants.kGearRatio);

    m_config =
        new TalonFXConfiguration()
            .withCurrentLimits(currentLimits)
            .withMotorOutput(motorOutput)
            .withFeedback(feedback);

    m_motor.getConfigurator().apply(m_config);

    m_motorVelocity = m_motor.getVelocity();
    Rotation2d.fromDegrees(90).getRadians();
    m_connectedMotor = m_motor.getConnectedMotor();
    m_motorCurrent = m_motor.getSupplyCurrent();
    m_motorStatorCurrent = m_motor.getStatorCurrent();
    m_motorVoltage = m_motor.getMotorVoltage();
    m_motorTemperature = m_motor.getDeviceTemp();

    // higher frequency for position
    BaseStatusSignal.setUpdateFrequencyForAll(100.0, m_motorPosition);

    // all of these are for logging so we can use a lower frequency
    BaseStatusSignal.setUpdateFrequencyForAll(
        75.0,
        m_connectedMotor,
        m_motorCurrent,
        m_motorVoltage,
        m_motorStatorCurrent,
        m_motorVelocity,
        m_motorTemperature);
  }


@Override
public void updateInputs(ShoryaInputs inputs) {
    if (!Constants.kUseBaseRefreshManager) {
        BaseStatusSignal.refreshAll(
            m_motorVoltage,
            m_motorCurrent,
            m_motorPosition,
            m_motorStatorCurrent,
            m_motorVelocity,
            m_motorTemperature);
    }

inputs.connected = m_connectedMotor.getValue() != ConnectedMotorValue.Unknown;

inputs.curr = getCurrAngle().getDegrees();
inputs.desired = m_desiredAngle.getDegrees();
inputs.atSetpoint = atSetpoint();
inputs.velocity = m_motorVelocity.getValue().in(RotationsPerSecond);
inputs.supplyCurrent = m_motorCurrent.getValue().in(Amps);
inputs.statorCurrent = m_motorStatorCurrent.getValue().in(Amps);
inputs.voltage = m_motorVoltage.getValue().in(Volts);
inputs.temperature = m_motorTemperature.getValue().in(Celsius);
}

@Override
public void setPIDFF(double kP, double kI, double kD, double kS) {
    m_motor
        .getConfigurator()
        .apply(
            m_config.Slot0.withKP(kP)
                .withKI(kI)
                .withKD(kD)
                .withKS(kS)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign),
            0.0);
}

@Override
 public void setDesiredAngle(Rotation2d angle) {
    double value = angle.getRadians();
    value =
        MathUtil.clamp(
            value,
            shoryaInstructioConstants.kWristMinAngle.getRadians(),
            ManipulatorConstants.kWristMaxAngle.getRadians());
    angle = Rotation2d.fromRadians(value);

    m_desiredAngle = angle;
    m_motor.setControl(m_positionControl.withPosition(angle.getRotations()));
  }

  private Rotation2d getCurrAngle() {
    return Rotation2d.fromRotations(m_motorPosition.getValueAsDouble());
  }

private boolean atSetpoint() {
    return Math.abs(m_desiredAngle.getDegrees() - getCurrAngle().getDegrees())
        < ShoryaConstants.kshoryaInstructionTolerance;
}

@Override
public void setCurrentLimits(double supplyLimit) {
    m_motor
        .getConfigurator()
        .apply(m_config.CurrentLimits.withSupplyCurrentLimit(supplyLimit), 0.0);
}
}

