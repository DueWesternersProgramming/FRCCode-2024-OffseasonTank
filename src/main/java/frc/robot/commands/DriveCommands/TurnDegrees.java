package frc.robot.commands.DriveCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.*;
/**
 * A command to drive the robot with joystick input (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes - actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.RunCommand}.
 */
public class TurnDegrees extends Command {
  private final DriveSubsystem m_drive;
  private double m_targetHeading;
  private double m_speed;
  private double m_accumulatedHeading;
  private double m_turnRadius;
  private boolean m_finished = false;
  private int m_direction;
  private SparkPIDController m_flSpeedPID;
  private SparkPIDController m_frSpeedPID;
  private double outerSpeed;
  private double innerSpeed;
  private double targetRSpeed;
  private double targetLSpeed;
  private double m_endPower = 0;

  public TurnDegrees(DriveSubsystem subsystem, double hoofs, double speed, int direction, double radius) {

    //setup method variables
    m_drive = subsystem;
    m_targetHeading = hoofs;
    m_speed = speed;
    m_direction = direction;
    m_turnRadius = radius;
    m_flSpeedPID = m_drive.getFrontLeftSparkMax().getPIDController();
    m_frSpeedPID = m_drive.getFrontRightSparkMax().getPIDController();
    
    //add drive requirements
    addRequirements(m_drive);
  }

  public TurnDegrees(DriveSubsystem subsystem, double hoofs, double speed, int direction, double radius, double endPower) {

    //setup method variables
    m_drive = subsystem;
    m_targetHeading = hoofs;
    m_speed = speed;
    m_direction = direction;
    m_turnRadius = radius;
    m_flSpeedPID = m_drive.getFrontLeftSparkMax().getPIDController();
    m_frSpeedPID = m_drive.getFrontRightSparkMax().getPIDController();
    m_endPower = endPower;

    //add drive requirements
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {

    System.out.println("Starting");
    
    //calculate the motor speeds required to turn a specific radius
    double robotRadius = .5*DriveConstants.kDriveWidth;
    double speedRatio = (m_turnRadius-robotRadius)/(m_turnRadius+robotRadius);
    outerSpeed = m_speed;
    innerSpeed = m_speed*speedRatio;

    System.out.println("outerSpeed: " + outerSpeed);
    System.out.println("innerSpeed: " + innerSpeed);

    if(m_direction == DriveConstants.kLeft) { 
      targetRSpeed = outerSpeed;
      targetLSpeed = innerSpeed;
    } else if(m_direction == DriveConstants.kRight) {
      targetRSpeed = innerSpeed;
      targetLSpeed = outerSpeed;
    }

    if(m_speed < 0) {
      m_direction *= -1;
    }

  }

  @Override
  public void execute() {

    //get the current heading
    m_accumulatedHeading = m_drive.getGyroAngle();

    //if the goal is reached, stop and set the command to finished
    if(m_direction == DriveConstants.kRight && m_accumulatedHeading > m_targetHeading) {
      m_frSpeedPID.setReference(m_endPower, ControlType.kVoltage);
      m_flSpeedPID.setReference(m_endPower, ControlType.kVoltage);
      m_finished = true;
    }else if(m_direction == DriveConstants.kLeft && m_accumulatedHeading < m_targetHeading) {
      m_frSpeedPID.setReference(m_endPower, ControlType.kVoltage);
      m_flSpeedPID.setReference(m_endPower, ControlType.kVoltage);
      m_finished = true;
    } else {
      m_flSpeedPID.setReference(targetLSpeed * DriveConstants.kMaxRPM, ControlType.kVelocity);
      m_frSpeedPID.setReference(targetRSpeed * DriveConstants.kMaxRPM, ControlType.kVelocity);
    }
  }

  @Override
  public boolean isFinished() {
    return m_finished;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Done");

    //m_drive.arcadeDrive(0, 0);
  }
}