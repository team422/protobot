package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeInputs {
    public double Position;
    public double VelocityRPS;
    public double Current;
    public double StatorCurrent;
    public double Voltage;
    public double Temperature;
    public boolean MotorIsConnected;
    public boolean Connected;
    public double Velocity;
    public double SupplyCurrent;
  }

  public void updateInputs(IntakeInputs inputs);

  public void setVoltage(double Voltage);

  public void setCurrentLimits(double supplyLimit);
}
