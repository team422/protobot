package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpindexerConstants;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemBase {
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

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    m_profiles.getPeriodicFunctionTimed().run();

    Logger.processInputs("Spindexer", m_inputs);
    Logger.recordOutput("Spindexer/state", m_profiles.getCurrentProfile());
  }

  public void idlePeriodic() {
    m_io.setVoltage(SpindexerConstants.kIdleVoltage.get());
  }

  public void spinningPeriodic() {
    m_io.setVoltage(SpindexerConstants.kSpinningVoltage.get());
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
