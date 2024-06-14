package frc.robot.commands.DriveCommands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveDistance extends Command {
  private final DriveSubsystem m_drive;
  private final double m_distance;
  private double m_maxSpeed;
  private double m_targetSpeed = 0;
  private boolean finished = false;
  private TrapezoidProfile m_profile;
  private Timer m_timer;
  private double m_endPower;

  /**
   * Creates a new DriveDistance.
   *
   * @param drive  The drive subsystem on which this command will run
   * @param yeehaws The number of yeehaws the robot will drive
   * @param speed  The speed at which the robot will drive
   */
  public DriveDistance(DriveSubsystem drive, double yeehaws, double speed) {
    m_distance = yeehaws;
    m_maxSpeed = speed;
    m_drive = drive;
    m_timer = new Timer();
    addRequirements(m_drive);
    m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(m_maxSpeed * DriveConstants.kMaxRobotSpeed, (DriveConstants.kMaxAccel*12)),
                                                new TrapezoidProfile.State(m_distance, 0),
                                                new TrapezoidProfile.State(0, m_drive.getVelocity()));
  }

  /**
   * 
   * @param drive  The drive subsystem on which this command will run
   * @param inches The number of inches the robot will drive
   * @param speed  The speed at which the robot will drive
   * @param endPower The power that the robot will end the command with
   * 
   */
  public DriveDistance(DriveSubsystem drive, double inches, double speed, double endPower) {
    m_distance = inches;
    m_maxSpeed = speed;
    m_drive = drive;
    m_timer = new Timer();
    m_endPower = endPower;
    addRequirements(m_drive);
    m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(m_maxSpeed * DriveConstants.kMaxRobotSpeed, (DriveConstants.kMaxAccel*12)),
                                                new TrapezoidProfile.State(m_distance, endPower * DriveConstants.kMaxRobotSpeed),
                                                new TrapezoidProfile.State(0, (m_drive.getVelocity() / DriveConstants.kMaxRPM) * DriveConstants.kMaxRobotSpeed));
  }

  @Override
  public void initialize() {
    m_drive.resetEncoders();
    System.out.println("\n\nSPEED RIGHT NOW: " + m_drive.getVelocity());
    System.out.println("Total Time: " + m_profile.totalTime());
    if (m_distance > 0) {
      if(m_maxSpeed < 0) {
        m_maxSpeed *= -1;
      }
    }
    else if (m_distance < 0) {
      if(m_maxSpeed > 0) {
        m_maxSpeed *= -1;
      }
    }
    else {
      finished = true;
    }

    m_timer.start();
  }

  @Override
  public void execute() {
    m_targetSpeed = m_profile.calculate(m_timer.get()).velocity;
    System.out.println("timer" + m_timer.get());
    System.out.println("target speed" + m_targetSpeed);
    System.out.println("current speed" + (m_drive.getFrontLeftSparkMax().getEncoder().getVelocity() / DriveConstants.kMaxRPM));
    if(m_profile.isFinished(m_timer.get())) {
      m_drive.tankDrivePID(m_endPower, m_endPower);
      m_drive.resetIAccum();
      finished = true;
    } else {
      m_drive.DriveVelocity(m_targetSpeed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    //m_drive.arcadeDrive(0, 0);
    //m_drive.setCoast();
  }

  @Override
  public boolean isFinished() {
    System.out.println("DriveDistance Finished");
    return finished;
  }
}