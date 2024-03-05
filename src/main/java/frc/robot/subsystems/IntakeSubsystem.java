package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.nio.ByteBuffer;
import edu.wpi.first.wpilibj.I2C;


public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotor, MotorType.kBrushless);
    private final DigitalInput colorSensor = new DigitalInput(1);


    //private I2C sensor = new I2C(null, 0);

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Sense color?", colorSensor.get());
    }

    public void setIntakeMotor(double velocity){
        if(!colorSensor.get()) {
            intakeMotor.set(0);
        } else {
            intakeMotor.set(velocity);
        }
    }

    public void stopIntakeMotor(){
        intakeMotor.set(0);
    }

    public boolean getColorSensor() {
        return colorSensor.get();
    }

}