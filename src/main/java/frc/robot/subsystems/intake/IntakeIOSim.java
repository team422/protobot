package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.IndexerConstants;

public class IntakeIOSim implements IntakeIO {
  private DCMotorSim m_sim;

  private double m_voltage = 0.0;

  public IntakeIOSim() {
    var plant =
        LinearSystemId.createDCMotorSystem(
            IndexerConstants.kSimGearbox, IndexerConstants.kSimMOI, IndexerConstants.kSimGearing);

    m_sim = new DCMotorSim(plant, IndexerConstants.kSimGearbox);
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    m_sim.setInputVoltage(m_voltage);
    m_sim.update(0.02);

    inputs.Position = m_sim.getAngularPositionRotations();
    inputs.VelocityRPS = m_sim.getAngularVelocityRPM() / 60;
    inputs.Current = m_sim.getCurrentDrawAmps();
    inputs.Voltage = m_voltage;

    // Don't matter for sim
    inputs.StatorCurrent = 0.0;
    inputs.MotorIsConnected = true;
  }

  @Override
  public void setVoltage(double Voltage) {
    m_voltage = Voltage;
  }

  @Override
  public void setCurrentLimits(double supplyLimit) {
    // Not needed for sim
  }
}