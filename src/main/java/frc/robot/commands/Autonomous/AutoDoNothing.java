package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveCommands.CalibrateGyro;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDoNothing extends SequentialCommandGroup{
    
    public AutoDoNothing(DriveSubsystem driveSubsystem) {
        addCommands(
            new CalibrateGyro(driveSubsystem),
            new WaitCommand(14));
      }
}