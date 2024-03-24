package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotor, MotorType.kBrushless);
    private final DigitalInput intakeLimit = new DigitalInput(8); //beam break


    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Note in?", intakeLimit.get());
    }

    public void setIntakeMotor(double velocity){
        intakeMotor.set(velocity);
    }

    public void setIntakeMotorLimited(double velocity, double dir){
        if(!intakeLimit.get() && dir == -1) {
            intakeMotor.set(0.02);
        } else {
            intakeMotor.set(velocity);
        }
    }

    public void stopIntakeMotor(){
        intakeMotor.set(0);
    }

    public boolean getIntakeLimit() {
        return intakeLimit.get();
    }
}