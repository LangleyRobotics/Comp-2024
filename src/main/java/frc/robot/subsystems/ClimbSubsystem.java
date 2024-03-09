package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimbSubsystem extends SubsystemBase {
    private final CANSparkMax climbMotorRight = new CANSparkMax(ClimbConstants.kClimbMotorRight, MotorType.kBrushless);
    private final CANSparkMax climbMotorLeft = new CANSparkMax(ClimbConstants.kClimbMotorLeft, MotorType.kBrushless);
    
    private final DigitalInput rightLimitSwitch = new DigitalInput(2);
    private final DigitalInput leftLimitSwitch = new DigitalInput(3);

    private int rightDir = 1;
    private int leftDir = 1;

    public ClimbSubsystem() {}

    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Right Climb Encoder Position", climbMotorRight.getEncoder().getPosition());
        SmartDashboard.putNumber("Left Climb Encoder Position", climbMotorLeft.getEncoder().getPosition());
    }

    public void setRightClimbMotor(double velocity){
        double vel = velocity * rightDir;
        if(!rightLimitSwitch.get() && vel < 0) {
            climbMotorRight.set(0);
            climbMotorRight.getEncoder().setPosition(0);
        } else if(vel > 0 && climbMotorRight.getEncoder().getPosition() >= ClimbConstants.encoderUpperLimit) {
            climbMotorRight.set(0);
        } else {
            climbMotorRight.set(vel);
        }
    }

    public void setLeftClimbMotor(double velocity){
        double vel = velocity * leftDir;
        if(!leftLimitSwitch.get() && vel < 0) {
            climbMotorLeft.set(0);
            climbMotorLeft.getEncoder().setPosition(0);
        } else if(vel > 0 && climbMotorLeft.getEncoder().getPosition() > ClimbConstants.encoderUpperLimit) {
            climbMotorRight.set(0);
        } else {
            climbMotorLeft.set(vel);
        }
    }

    public boolean getleftLimitSwitch(){
        return !leftLimitSwitch.get();
    }

    public void switchDir(char c) {
        if(c == 'r') {
            rightDir *= -1;
        } else if (c == 'l') {
            leftDir *= -1;
        }
    }

    public void stopClimbMotor(){
        climbMotorRight.set(0);
        climbMotorLeft.set(0);
    }

}