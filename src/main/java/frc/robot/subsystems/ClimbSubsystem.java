package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
//If we ever use a limit switch
import edu.wpi.first.wpilibj.DigitalInput;

public class ClimbSubsystem extends SubsystemBase {
    private final CANSparkMax climbMotorRight = new CANSparkMax(ClimbConstants.kClimbMotorRight, MotorType.kBrushless);
    private final CANSparkMax climbMotorLeft = new CANSparkMax(ClimbConstants.kClimbMotorLeft, MotorType.kBrushless);
    
    private final DigitalInput rightLimitSwitch = new DigitalInput(1);
    private final DigitalInput leftLimitSwitch = new DigitalInput(2);

    public ClimbSubsystem() {}

    
    @Override
    public void periodic() {

    }

    public void setRightClimbMotor(double velocity){
        if(!rightLimitSwitch.get()) {
            climbMotorRight.set(0);
        } else {
            climbMotorRight.set(velocity);
        }
    }

    public void setLeftClimbMotor(double velocity){
        if(!leftLimitSwitch.get()) {
            climbMotorLeft.set(0);
        } else {
            climbMotorLeft.set(velocity);
        }
    }

    public void stopClimbMotor(){
        climbMotorRight.set(0);
        climbMotorLeft.set(0);
    }

}