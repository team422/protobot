package frc.robot.subsystems.shoryaInstruction;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface shoryaInstructionIO {
  @AutoLog
  public static class ShoryaInputs {
    public double curr;
    public double desired;
    public boolean atSetpoint;
    public double velocity;
    public double current;
    public double supplyCurrent;
    public double statorCurrent;
    public double voltage;
    public double temperature;
    public boolean connected;
  }

  public void updateInputs(ShoryaInputs inputs);

  public void setPIDFF(double kP, double kI, double kD, double kS);

  public void setDesiredAngle(Rotation2d angle);

  public void setCurrentLimits(double supplyLimit);
}
