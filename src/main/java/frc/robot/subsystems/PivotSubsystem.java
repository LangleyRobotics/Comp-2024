package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.apriltag.AprilTag;

//If we ever use a limit switch
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.MathMethods;

public class PivotSubsystem extends SubsystemBase {
    private final CANSparkMax pivotMotorRight = new CANSparkMax(PivotConstants.kPivotMotorRight, MotorType.kBrushless);
    private final CANSparkMax pivotMotorLeft = new CANSparkMax(PivotConstants.kPivotMotorLeft, MotorType.kBrushless);
    private final DutyCycleEncoder pivotAbsEncoder = new DutyCycleEncoder(1);

    //Don't think we need these???
    private final ArmFeedforward pivotFeedForward = new ArmFeedforward(PivotConstants.kS_Pivot, PivotConstants.kG_Pivot, PivotConstants.kV_Pivot, PivotConstants.kA_Pivot);
    private final PIDController pivotPIDController = new PIDController(PivotConstants.kP_Pivot, PivotConstants.kI_Pivot, PivotConstants.kD_Pivot);

    double absEncoderRaw = 0;


    public PivotSubsystem() {}


    @Override
    public void periodic() {

    }

    public void setPivotMotor(double velocity){

        try {
            if ((MathMethods.signDouble(velocity) == -1 && getPivotAbsEncoder() < PivotConstants.kMaxPivotPosition) || 
                    (MathMethods.signDouble(velocity) == 1 && getPivotAbsEncoder() > PivotConstants.kMinPivotPosition)) {
                    //motors are facing opposite directions
                    pivotMotorRight.set(velocity);
                    pivotMotorLeft.set(-velocity);
                } else {
                    pivotMotorRight.set(0.0);
                    pivotMotorLeft.set(0.0);
                }
        } catch(Exception e) {
            System.out.println("Error: Pivot Motor is Set to a value out of valid range [-1.0, 1.0]");
        }
    }

    public void setPivotMotorNoBounds(double velocity) {
        pivotMotorRight.set(velocity);
        pivotMotorLeft.set(-velocity);
    }

    public void stopPivotMotor(){
        pivotMotorRight.set(0);
        pivotMotorLeft.set(0);
    }

    //returns the value of the pivot encoder (position of the arm angle)
    public double getPivotAbsEncoder() {
        pivotAbsEncoder.setPositionOffset(0);
        absEncoderRaw = pivotAbsEncoder.getAbsolutePosition() + PivotConstants.kPivottOffset;
        if (absEncoderRaw < PivotConstants.kPivotEncoderBreakpoint) {
            return (absEncoderRaw + 1);
        } else {
            return absEncoderRaw;
        }
    }

    //Feedforward and PID controller for pivot
    //
    // public void pivotWithFeedforwardPID(double desPosition, double desVelocity, double desAccel) {
    //     pivotMotorRight.setVoltage(pivotFeedForward.calculate(desPosition, desVelocity, desAccel)
    //         + pivotPIDController.calculate(pivotAbsEncoder.getRate(), leftVelocitySetpoint));
    //     rightMotor.setVoltage(feedForward.calculate(rightVelocitySetpoint)
    //         + rightPID.calculate(rightEncoder.getRate(), rightVelocitySetpoint));
    //   }

}