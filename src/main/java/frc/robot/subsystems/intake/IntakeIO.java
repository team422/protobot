package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeInputs {
    public double position;
    public double velocityRPS;
    public double supplyCurrent;
    public double statorCurrent;
    public double voltage;
    public double temperature;
    public boolean connected;
  }

  public void updateInputs(IntakeInputs inputs);

  public void setVoltage(double volts);

  public void setCurrentLimits(double supplyLimit);
}
