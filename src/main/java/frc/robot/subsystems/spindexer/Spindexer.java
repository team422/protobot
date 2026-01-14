package frc.robot.subsystems.spindexer;

import frc.robot.Constants.SpindexerConstants;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Spindexer {
  private SpindexerIO m_spindexerIO;
  public final SpindexerInputsAutoLogged m_inputs = new SpindexerInputsAutoLogged();

  private SubsystemProfiles<SpindexerState> m_profiles;

  public static enum SpindexerState {
    kIdle,
    kSpinning,
    kScoring,
  }

  public Spindexer(SpindexerIO spindexerIO) {
    m_spindexerIO = spindexerIO;
    Map<SpindexerState, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(SpindexerState.kIdle, this::idlePeriodic);
    periodicHash.put(SpindexerState.kSpinning, this::spinningPeriodic);
    periodicHash.put(SpindexerState.kScoring, this::scoringPeriodic);

    m_profiles = new SubsystemProfiles<>(periodicHash, SpindexerState.kIdle);
  }

  public void periodic() {
    m_spindexerIO.updateInputs(m_inputs);
    Logger.processInputs("Spindexer", m_inputs);
    Logger.recordOutput("Spindexer Velocity", m_inputs.velocityRPS);
  }

  public void idlePeriodic() {
    m_spindexerIO.setVoltage(SpindexerConstants.kIdleVoltage);
  }

  public void spinningPeriodic() {
    m_spindexerIO.setVoltage(SpindexerConstants.kSpinningVoltage);
  }

  public void scoringPeriodic() {
    m_spindexerIO.setVoltage(SpindexerConstants.kScoringVoltage);
  }

  public void updateState(SpindexerState state) {
    m_profiles.setCurrentProfile(state);
  }

  public SpindexerState getCurrentState() {
    return m_profiles.getCurrentProfile();
  }

  public double getCurrentVelocity() {
    return m_inputs.velocityRPS;
  }
}
