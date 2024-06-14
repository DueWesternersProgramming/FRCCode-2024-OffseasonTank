package frc.robot.commands.Autonomous.Paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveCommands.*;
import frc.robot.subsystems.DriveSubsystem;

public class PathAnyScore extends SequentialCommandGroup {

    
    /**
     * 
     * @param m_drive
     * @param m_arm
     * @param m_armBase
     * @param m_claw
     * @param m_turret
     */
    public PathAnyScore(DriveSubsystem m_drive) {
        addCommands(
        new CalibrateGyro(m_drive),
        new DriveDistance(m_drive, 10, 0.07),
        new WaitCommand(0.5)
        );

    }

}
