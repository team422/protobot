package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Ports;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.oi.DriverControls;
import frc.robot.oi.DriverControlsPS5;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.Spindexer.SpindexerState;
import frc.robot.subsystems.spindexer.SpindexerIOKraken;
import frc.robot.subsystems.spindexer.SpindexerIOSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  private Drive m_drive;
  private Spindexer m_spindexer;

  // Controller
  private DriverControls m_controller;

  // Dashboard inputs
  private LoggedDashboardChooser<Command> m_autoChooser;

  public RobotContainer() {
    configureSubsystems();
    configureCommands();
    configureControllers();
    configureBindings();
  }

  public void configureSubsystems() {
    if (RobotBase.isReal()) {
      m_drive =
          new Drive(
              new GyroIOPigeon2(),
              new ModuleIOTalonFX(0),
              new ModuleIOTalonFX(1),
              new ModuleIOTalonFX(2),
              new ModuleIOTalonFX(3));
      m_spindexer = new Spindexer(new SpindexerIOKraken(Ports.kSpindexer, Ports.kMainCanivoreName));
    } else {
      m_drive =
          new Drive(
              new GyroIOPigeon2(),
              new ModuleIOSim(),
              new ModuleIOSim(),
              new ModuleIOSim(),
              new ModuleIOSim());
      m_spindexer = new Spindexer(new SpindexerIOSim());
    }
  }

  public void configureCommands() {
    RobotState.startInstance(m_drive);
  }

  public void configureControllers() {
    m_controller = new DriverControlsPS5(0);
  }

  public void configureBindings() {
    m_drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            m_drive,
            m_controller::getForward,
            m_controller::getStrafe,
            m_controller::getTurn,
            false));
    m_controller
        .resetFieldCentric()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_drive.setPose(new Pose2d());
                }));

    // will be removed once it goes on the actual robot (always spins)
    m_controller
        .spin()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (m_spindexer.getCurrentState() == SpindexerState.kSpinning) {
                    m_spindexer.updateState(SpindexerState.kIdle);
                  } else {
                    m_spindexer.updateState(SpindexerState.kSpinning);
                  }
                }));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.get();
  }

  public String getSelectedAuto() {
    return m_autoChooser.getSendableChooser().getSelected();
  }
}
