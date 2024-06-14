// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autonomous.*;
import frc.robot.commands.DriveCommands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final GenericHID driverController;
  
  SendableChooser<Command> m_autoPositionChooser = new SendableChooser<>();
  
  PowerDistribution PDP = new PowerDistribution(16, ModuleType.kCTRE);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    driverController = new GenericHID(OperatorConstants.kDriverControllerPort);
    
    // Configure the trigger bindings
    configureButttonBindings();
    driveSubsystem.setDefaultCommand(new ArcadeDrive(driveSubsystem,
    () -> driverController.getRawAxis(1),
    () -> driverController.getRawAxis(4)));

    m_autoPositionChooser.addOption("Do Nothing", new AutoDoNothing(driveSubsystem));
    m_autoPositionChooser.addOption("Calibrate Gryo", new AutoCalibrateGyro(driveSubsystem));

    Shuffleboard.getTab("Autonomous").add(m_autoPositionChooser);

    Shuffleboard.getTab("Power").add(PDP);
  }



  private void configureButttonBindings() {
    new POVButton(driverController, 0).onTrue(new TurnDegrees(driveSubsystem, driveSubsystem.getHeading()-driveSubsystem.getYaw(), 0.5, DriveConstants.kLeft, 0));
    new JoystickButton(driverController, 3);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoPositionChooser.getSelected();
  }
}
