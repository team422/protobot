package frc.robot.subsystems.spindexer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.SpindexerConstants;

public class SpindexerIOSim implements SpindexerIO{

    private PIDController m_controller = new PIDController(0, 0, 0);
    private DCMotorSim m_sim;
    private double m_voltage = 0.0;

    public SpindexerIOSim() {

        var plant = LinearSystemId.createDCMotorSystem(SpindexerConstants.kSimGearbox, SpindexerConstants.kSimMOI, SpindexerConstants.kSimGearing);
        m_sim = new DCMotorSim(plant, SpindexerConstants.kSimGearbox);

    }

    @Override
    public void updateInputs(SpindexerInputs inputs) {
        m_sim.setInputVoltage(m_voltage);
        m_sim.update(0.02);

        inputs.velocityRPS = m_sim.getAngularVelocityRPM() / 60;
        inputs.accelerationRPSSq = Units.radiansToRotations(m_sim.getAngularAccelerationRadPerSecSq());
        inputs.current = m_sim.getCurrentDrawAmps();
        inputs.voltage = m_voltage;
        
        inputs.statorCurrent = 0.0;
        inputs.temperature = 0.0;
        inputs.motorIsConnected = false;
    }

    @Override
    public void setVoltage(double voltage) {
        m_voltage = voltage;
        
    }

    @Override
    public void setCurrentLimits(double supplyLimit) {
        // Listen here boy sim don't need this bro - Signed: Me
        
    }

    @Override
    public void setPIDFF(int slot, double kP, double kI, double kD, double kS) {
        if (slot == 0) {
            m_controller.setPID(kP, kI, kD);
          }
    }
    @Override
    public void setSlot(int slot) {
        // Buddy not this in sim either okay
        
    }

}
