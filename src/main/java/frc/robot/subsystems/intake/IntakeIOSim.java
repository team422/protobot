package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.IndexerConstants;

public class IntakeIOSim implements IntakeIO {
  private DCMotorSim m_Sim;

  private double m_Voltage = 0.0;

  public IntakeIOSim() {
    var plant =
        LinearSystemId.createDCMotorSystem(
            IndexerConstants.kSimGearbox, IndexerConstants.kSimMOI, IndexerConstants.kSimGearing);

    m_Sim = new DCMotorSim(plant, IndexerConstants.kSimGearbox);
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    m_Sim.setInputVoltage(m_Voltage);
    m_Sim.update(0.02);

    inputs.Position = m_Sim.getAngularPositionRotations();
    inputs.VelocityRPS = m_Sim.getAngularVelocityRPM() / 60;
    inputs.Current = m_Sim.getCurrentDrawAmps();
    inputs.Voltage = m_Voltage;

    //m_topSim.setInputVoltage(m_topVoltage);
    //m_topSim.update(0.02);

    //inputs.topPosition = m_topSim.getAngularPositionRotations();
    //inputs.topVelocityRPS = m_topSim.getAngularVelocityRPM() / 60;
    //inputs.topCurrent = m_topSim.getCurrentDrawAmps();
    //inputs.topVoltage = m_topVoltage;

    // these don't matter in sim
    inputs.StatorCurrent = 0.0;
    inputs.MotorIsConnected = false;
  }

  @Override
  public void setVoltage(double Voltage) {
    m_Voltage = Voltage;
    //m_topVoltage = topVoltage;
  }

  @Override
  public void setCurrentLimits(double supplyLimit) {
    // Not needed for simulation
  }
}