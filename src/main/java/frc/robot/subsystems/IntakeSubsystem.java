package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotor, MotorType.kBrushless);

    @Override
    public void periodic() {

    }

    public void setIntakeMotor(double velocity){
        intakeMotor.set(velocity);
    }

    public void stopIntakeMotor(){
        intakeMotor.set(0);
    }

}