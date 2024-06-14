// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants.*;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  private CANSparkMax motor1L, motor2L, motor1R, motor2R;
  private RelativeEncoder encoderL, encoderR;
  private AHRS ahrs;
  private DifferentialDrive m_drive;
  private double PID_P = DriveConstants.kDefaultP;
  private double PID_I = DriveConstants.kDefaultI;
  private double PID_D = DriveConstants.kDefaultD;
  private double pitchRotationOffset, rotationOffset, turnTargetMovementTotal;
  private int turnTargetMovementDirection;
  private int speed = 1; // 0 = slow, 1 = normal, 2 = fast


  /** Creates a new DriveSubsystem.
   * @todo Fix error catching
   */

  public DriveSubsystem() {
    try {
      motor1L = new CANSparkMax(DriveConstants.kLeft1MotorPort,CANSparkMax.MotorType.kBrushless);
      motor2L = new CANSparkMax(DriveConstants.kLeft2MotorPort,CANSparkMax.MotorType.kBrushless);
      motor1R = new CANSparkMax(DriveConstants.kRight1MotorPort,CANSparkMax.MotorType.kBrushless);
      motor2R = new CANSparkMax(DriveConstants.kRight2MotorPort,CANSparkMax.MotorType.kBrushless);
      motor2L.follow(motor1L);
      motor2R.follow(motor1R);
      motor1R.setInverted(true);
      motor2R.setInverted(true);
      motor1L.setInverted(false);
      motor2L.setInverted(false);
      m_drive = new DifferentialDrive(motor1L, motor1R);
      encoderL = motor1L.getEncoder();
      encoderR = motor1R.getEncoder();
      encoderL.setVelocityConversionFactor(0.00398982267005903741284755709676/10.71);
      encoderR.setVelocityConversionFactor(0.00398982267005903741284755709676/10.71);
      encoderL.setPositionConversionFactor(0.00398982267005903741284755709676/10.71);
      encoderR.setPositionConversionFactor(0.00398982267005903741284755709676/10.71);
      resetEncoders();
      updatePID();
    }
    catch (Exception e){
      System.out.println("Motor setup error: " + e + "\n");
      e.printStackTrace();
    }
    try {
      ahrs = new AHRS(SerialPort.Port.kMXP);
    }
    catch (Exception e){
      System.out.println("Gyro error: " + e + "\n");
    }
  }

  public void arcadeDrive(double drive, double rot){
    if (speed == 1){
      m_drive.arcadeDrive(drive * DriveConstants.kNormalSpeedMultiplier, rot * DriveConstants.kNormalSpeedMultiplier);
    }
    else if (speed == 2){
      m_drive.arcadeDrive(drive * DriveConstants.kFastSpeedMultiplier, rot * DriveConstants.kFastSpeedMultiplier);
    }
    else if (speed == 3){
      m_drive.arcadeDrive(drive * DriveConstants.kAutoSpeedMultiplier, rot * DriveConstants.kAutoSpeedMultiplier);
    }
    else {
      m_drive.arcadeDrive(drive * DriveConstants.kSlowSpeedMultiplier, rot * DriveConstants.kSlowSpeedMultiplier);
    }
  }

  public boolean toggleBrake(){
    if (motor1L.getIdleMode().equals(IdleMode.kCoast)){
      motor1L.setIdleMode(IdleMode.kBrake);
      motor2L.setIdleMode(IdleMode.kBrake);
      motor1R.setIdleMode(IdleMode.kBrake);
      motor2R.setIdleMode(IdleMode.kBrake);
      return true;
    }
    else{
      motor1L.setIdleMode(IdleMode.kCoast);
      motor2L.setIdleMode(IdleMode.kCoast);
      motor1R.setIdleMode(IdleMode.kCoast);
      motor2R.setIdleMode(IdleMode.kCoast);
      return false;
    }
  }

  private void updatePID() {
    motor1L.getPIDController().setP(PID_P);
    motor1R.getPIDController().setP(PID_P);
    motor1L.getPIDController().setI(PID_I);
    motor1R.getPIDController().setI(PID_I);
    motor1L.getPIDController().setD(PID_D);
    motor1R.getPIDController().setD(PID_D);
  }

  public void tankDrivePID(double left, double right) {
    motor1L.getPIDController().setReference(left * DriveConstants.kMaxRPM , ControlType.kVelocity);
    motor1R.getPIDController().setReference(right * DriveConstants.kMaxRPM , ControlType.kVelocity);
  }

  public void driveVelocity(double velocity) {
    System.out.println((velocity / DriveConstants.kMaxRobotSpeed) * DriveConstants.kMaxRPM);
    motor1L.getPIDController().setReference((velocity / DriveConstants.kMaxRobotSpeed) * DriveConstants.kMaxRPM , ControlType.kVelocity);
    motor1R.getPIDController().setReference((velocity / DriveConstants.kMaxRobotSpeed) * DriveConstants.kMaxRPM , ControlType.kVelocity);
  }

  public void setDistancePerPulse() {
    encoderL.setPositionConversionFactor((DriveConstants.kGearRatio)*Math.PI*DriveConstants.kWheelDiameterInches);
    encoderR.setPositionConversionFactor((DriveConstants.kGearRatio)*Math.PI*DriveConstants.kWheelDiameterInches);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(encoderL.getVelocity(), encoderR.getVelocity());
  }

  public void DriveVelocity(double velocity) {
    //forward and turning variables for calculation
    double m_fwd = velocity;
    System.out.println((m_fwd / DriveConstants.kMaxRobotSpeed) * DriveConstants.kMaxRPM);
    motor1L.getPIDController().setReference((m_fwd / DriveConstants.kMaxRobotSpeed) * DriveConstants.kMaxRPM , ControlType.kVelocity);
    motor1R.getPIDController().setReference((m_fwd / DriveConstants.kMaxRobotSpeed) * DriveConstants.kMaxRPM , ControlType.kVelocity);
  }

  public void setCustomRotation() {
    pitchRotationOffset = ahrs.getPitch();
    rotationOffset = ahrs.getRoll();
  }

  public void setTurnTargetMovementResult(double result) {
    turnTargetMovementTotal = Math.abs(result);
    if (result > 0){
      turnTargetMovementDirection = 1;
    }
    else {
      turnTargetMovementDirection = -1;
    }
  }

  public double getTurnTargetMovementResult(){
    return turnTargetMovementTotal;
  }

  public int getTurnTargetMovementDirection(){
    return turnTargetMovementDirection;
  }

  public CANSparkMax getFrontLeftSparkMax() {
    return motor1L;
  }

  public CANSparkMax getFrontRightSparkMax() {
    return motor1R;
  }

  public double getLeftEncoderPosition(){
    return encoderL.getPosition();
  }

  public double getRightEncoderPosition(){
    return encoderR.getPosition();
  }

  public void resetEncoders(){
    encoderL.setPosition(0.0);
    encoderR.setPosition(0.0);
  }

  public double getAverageEncoderDistance() {
    return (encoderL.getPosition() + encoderR.getPosition()) / 2.0;
  }

  public RelativeEncoder getLeftEncoder() {
    return encoderL;
  }

  public RelativeEncoder getRightEncoder() {
    return encoderR;
  }

  public void setBrake() {
    motor1L.setIdleMode(IdleMode.kBrake);
    motor2L.setIdleMode(IdleMode.kBrake);
    motor1R.setIdleMode(IdleMode.kBrake);
    motor2R.setIdleMode(IdleMode.kBrake);
  }

  public void setCoast() {
    motor1L.setIdleMode(IdleMode.kCoast);
    motor2L.setIdleMode(IdleMode.kCoast);
    motor1R.setIdleMode(IdleMode.kCoast);
    motor2R.setIdleMode(IdleMode.kCoast);
  }

  private boolean isBrake(){
    if (motor1L.getIdleMode().equals(IdleMode.kBrake)){
      return true;
    }
    else {
      return false;
    }
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  public double getGyroAngle() {
    return (ahrs.getAngle());
  }

  public void calibrateGyro() {
    ahrs.reset();
    while(ahrs.isCalibrating()) {
    }
  }

  public boolean gyroIsCalibrating() {
    return ahrs.isCalibrating();
  }

  public void zeroHeading() {
    ahrs.reset();
  }

  public double getHeading() {
    return ahrs.getRotation2d().getDegrees();
  }

  public double getYaw() {
    return ahrs.getYaw();
  }

  public double getRotation() {
    if (rotationOffset > 0){
      return ahrs.getRoll() - rotationOffset;
    }
    else if (rotationOffset < 0){
      return ahrs.getRoll() + rotationOffset;
    }
    return ahrs.getRoll();
  }

  public double getPitch() {
    if (pitchRotationOffset > 0){
      return ahrs.getPitch() + pitchRotationOffset;
    }
    else if (pitchRotationOffset < 0){
      return ahrs.getPitch() - pitchRotationOffset;
    }
    return ahrs.getPitch();
  }

  public double getTurnRate() {
    return ahrs.getRate();
  }

  public void restorePID_Defaults() {
    PID_P = DriveConstants.kDefaultP;
    PID_I = DriveConstants.kDefaultI;
    PID_D = DriveConstants.kDefaultD;
    updatePID();
  }

  public void setPID_P(double p) {
    PID_P = p;
    updatePID();
  }

  public void setPID_I(double i) {
    PID_I = i;
    updatePID();
  }

  public void setPID_D(double d) {
    PID_D = d;
    updatePID();
  }

  public void resetIAccum() {
    motor1L.getPIDController().setIAccum(0);
    motor1R.getPIDController().setIAccum(0);
  }

  //get the average speed of the motors
  public double getVelocity() {
    return (getRightEncoder().getVelocity() + getRightEncoder().getVelocity()) / 2;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_drive.feed();
    SmartDashboard.putNumber("Angle", ahrs.getAngle());
    SmartDashboard.putNumber("Roll", ahrs.getRoll());
    SmartDashboard.putNumber("Custom Roll", getRotation());
    SmartDashboard.putNumber("Pitch", ahrs.getPitch());
    SmartDashboard.putNumber("Encoder L", encoderL.getPosition());
    SmartDashboard.putNumber("Encoder R", encoderR.getPosition());
    SmartDashboard.putNumber("Speed L", motor1L.get());
    SmartDashboard.putNumber("Speed R", motor1R.get());
    SmartDashboard.putNumber("Motor1L Current", motor1L.getOutputCurrent());
    SmartDashboard.putNumber("Motor2L Current", motor2L.getOutputCurrent());
    SmartDashboard.putNumber("Motor1R Current", motor1R.getOutputCurrent());
    SmartDashboard.putNumber("Motor2R Current", motor2R.getOutputCurrent());
    SmartDashboard.putNumber("Fast Mode", speed);
    SmartDashboard.putBoolean("Brake Mode", isBrake());
    SmartDashboard.putNumber("Custom Pitch", getPitch());
    SmartDashboard.putNumber("Offset", pitchRotationOffset);
  }
}