package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;





public class MechanismSubsystem extends SubsystemBase{
    
    CANSparkMax AmpMotor = new CANSparkMax(10, MotorType.kBrushless);  // WRIST PORT NOT SET
    RelativeEncoder ampEncoder = AmpMotor.getEncoder();

    public MechanismSubsystem(){
        AmpMotor.setIdleMode(IdleMode.kBrake);
        AmpMotor.setInverted(true);
        resetEncoder();
    }

    public void runWrist(double speed){
        AmpMotor.set(speed);
    }

    public double getSpeed(){
        return AmpMotor.get();
    }

    public double getEncoderPosition() {
        return ampEncoder.getPosition();
    }

    public void resetEncoder() {
        ampEncoder.setPosition(0.0);
    }

    @Override
    public void periodic() {
       
    }

}