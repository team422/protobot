package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {
  @AutoLog
  public static class SpindexerInputs {
    public double velocityRPS;
    public double accelerationRPSSq;
    public double current;
    public double statorCurrent;
    public double voltage;
    public double temperature;
    public boolean motorIsConnected;
  }

  public void updateInputs(SpindexerInputs inputs);

  public void setVoltage(double voltage);

  public void setCurrentLimits(double supplyLimit);
}
