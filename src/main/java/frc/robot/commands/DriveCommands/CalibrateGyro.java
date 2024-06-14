package frc.robot.commands.DriveCommands;

import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/** An intake command that uses the driveSubsystem. */
public class CalibrateGyro extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private boolean finished = false;

  /**
   * Creates a new StartIntake command.
   *
   * @param intakeSubsystem The subsystem used by this command.
   */
  public CalibrateGyro(DriveSubsystem driveSubsystem) {
    m_driveSubsystem = driveSubsystem;
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    m_driveSubsystem.zeroHeading();
    m_driveSubsystem.calibrateGyro();
    System.out.println("Calibrating");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_driveSubsystem.gyroIsCalibrating()){ // Original 0
      finished = true;
      System.out.println("done calibrating");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.setCustomRotation();
    System.out.println("Stopped");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}