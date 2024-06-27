// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ArcadeDrive extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private DoubleSupplier m_left, m_right;
  private double m_leftModified, m_rightModified;

  /**
   * Creates a new TankDrive command.
   *
   * @param driveSubsystem The subsystem used by this command.
   */
  public ArcadeDrive(DriveSubsystem driveSubsystem, DoubleSupplier left, DoubleSupplier right) {
    m_driveSubsystem = driveSubsystem;
    m_left = left;
    m_right = right;
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_driveSubsystem.setCoast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_leftModified = m_left.getAsDouble();
    m_rightModified = m_right.getAsDouble();
    if (Math.abs(m_left.getAsDouble()) < OperatorConstants.kControllerDeadZone){
      m_leftModified = 0.0;
    }
    if (Math.abs(m_right.getAsDouble()) < OperatorConstants.kControllerDeadZone){
      m_rightModified = 0.0;
    }
    m_driveSubsystem.arcadeDrive(m_leftModified, -m_rightModified);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.arcadeDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}