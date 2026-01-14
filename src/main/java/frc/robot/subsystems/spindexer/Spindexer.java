package frc.robot.subsystems.spindexer;

import frc.robot.Constants.SpindexerConstants;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Spindexer {
  private SpindexerIO m_io;
  public final SpindexerInputsAutoLogged m_inputs = new SpindexerInputsAutoLogged();

  private SubsystemProfiles<SpindexerState> m_profiles;

  public static enum SpindexerState {
    kIdle,
    kSpinning,
  }

  public Spindexer(SpindexerIO spindexerIO) {
    m_io = spindexerIO;
    Map<SpindexerState, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(SpindexerState.kIdle, this::idlePeriodic);
    periodicHash.put(SpindexerState.kSpinning, this::spinningPeriodic);

    m_profiles = new SubsystemProfiles<>(periodicHash, SpindexerState.kIdle);
  }

  public void periodic() {
    m_io.updateInputs(m_inputs);
    m_profiles.getPeriodicFunctionTimed().run();

    Logger.processInputs("Spindexer", m_inputs);
    Logger.recordOutput("Spindexer State", m_profiles.getCurrentProfile());
  }

  public void idlePeriodic() {
    m_io.setVoltage(SpindexerConstants.kIdleVoltage);
  }

  public void spinningPeriodic() {
    m_io.setVoltage(SpindexerConstants.kSpinningVoltage);
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
