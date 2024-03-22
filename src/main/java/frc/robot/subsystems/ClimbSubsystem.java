package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.PivotConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimbSubsystem extends SubsystemBase {
    private final CANSparkMax climbMotorRight = new CANSparkMax(ClimbConstants.kClimbMotorRight, MotorType.kBrushless);
    private final CANSparkMax climbMotorLeft = new CANSparkMax(ClimbConstants.kClimbMotorLeft, MotorType.kBrushless);
    
    private final DutyCycleEncoder rightAbsEncoder = new DutyCycleEncoder(2);
    private final DutyCycleEncoder leftAbsEncoder = new DutyCycleEncoder(3);

    private double rightPos = 0.0;
    // private double leftPos = 0.0;

    private int rightDir = 1;
    private int leftDir = 1;

    public ClimbSubsystem() {
        rightAbsEncoder.setDistancePerRotation(ClimbConstants.disPerRot);
        leftAbsEncoder.setDistancePerRotation(ClimbConstants.disPerRot);

        rightPos = Preferences.getDouble(ClimbConstants.rightPosKey, rightPos);
        // leftPos = Preferences.getDouble(ClimbConstants.leftPosKey, leftPos);

        rightAbsEncoder.reset();
        leftAbsEncoder.reset();
    }

    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Right Climb Encoder Position", getRightAbsEncoder());
        SmartDashboard.putNumber("Left Climb Encoder Position", getLeftAbsEncoder());
    }

    public void setRightClimbMotor(double velocity){
        double vel = velocity * rightDir;

        if((getRightAbsEncoder() < ClimbConstants.rightLowerLimit && vel < 0)
            || (getRightAbsEncoder() > ClimbConstants.rightUpperLimit && vel > 0)) {
            //at the bottom or top
            climbMotorRight.set(0);
        } else {
            climbMotorRight.set(vel);
        }
    }

    public void setLeftClimbMotor(double velocity){
        double vel = velocity * leftDir;
        
        if((getLeftAbsEncoder() < ClimbConstants.leftLowerLimit && vel < 0)
            || (getLeftAbsEncoder() > ClimbConstants.leftUpperLimit && vel > 0)) {
            //at the bottom or top
            climbMotorLeft.set(0);
        } else {
            climbMotorLeft.set(vel);
        }
    }

    //returns the value of the right climb encoder
    public double getRightAbsEncoder() {
        rightAbsEncoder.setDistancePerRotation(ClimbConstants.disPerRot);
        return rightAbsEncoder.getDistance() + rightPos;
    }

    //returns the value of the left climb encoder
    public double getLeftAbsEncoder() {
        leftAbsEncoder.setDistancePerRotation(ClimbConstants.disPerRot);
        // return leftAbsEncoder.getDistance() + leftPos;
        return leftAbsEncoder.getDistance();
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

        Preferences.setDouble(ClimbConstants.rightPosKey, getRightAbsEncoder());
        // Preferences.setDouble(ClimbConstants.leftPosKey, getLeftAbsEncoder());
    }

}