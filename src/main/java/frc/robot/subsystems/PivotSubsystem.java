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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.MathMethods;

public class PivotSubsystem extends SubsystemBase {
    private final CANSparkMax pivotMotorRight = new CANSparkMax(PivotConstants.kPivotMotorRight, MotorType.kBrushless);
    private final CANSparkMax pivotMotorLeft = new CANSparkMax(PivotConstants.kPivotMotorLeft, MotorType.kBrushless);
    private final DutyCycleEncoder pivotAbsEncoder = new DutyCycleEncoder(1);

    private final ArmFeedforward pivotFeedForward = new ArmFeedforward(PivotConstants.kS_Pivot, PivotConstants.kG_Pivot, PivotConstants.kV_Pivot, PivotConstants.kA_Pivot);
    private final PIDController pivotPIDController = new PIDController(PivotConstants.kP_Pivot, PivotConstants.kI_Pivot, PivotConstants.kD_Pivot);

    double absEncoderRaw = 0;
    double offset = 0;
    

    public PivotSubsystem() {
        // Configures the encoder to return a distance for every rotation
        pivotAbsEncoder.setDistancePerRotation(PivotConstants.disPerRot);
        offset = pivotAbsEncoder.getDistance() - PivotConstants.kPivotOffset;
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Encoder", getPivotAbsEncoder());
        SmartDashboard.putBoolean("Pivot Encoder Connected?", pivotAbsEncoder.isConnected());
        SmartDashboard.putNumber("Pivot Right Voltage", pivotMotorRight.getBusVoltage());
        SmartDashboard.putNumber("Pivot Left Voltage", pivotMotorLeft.getBusVoltage());

    }

    //Manually controlling angle of pivot (operator controller during teleop)
    public void setPivotMotor(double velocity){

        try {
            if (getPivotAbsEncoder() < PivotConstants.kMaxPivotPosition || 
                getPivotAbsEncoder() > PivotConstants.kMinPivotPosition) {
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

    // public void useOutput(double output, double setpoint) {
    //     pivotMotorRight.setVoltage(output + pivotFeedForward.calculate(setpoint, pivotAbsEncoder.));
    // }

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
        pivotAbsEncoder.setDistancePerRotation(PivotConstants.disPerRot);
        return pivotAbsEncoder.getDistance() - offset;
    }

    //Send arm to a specific setpoint
    public void goToSetpoint(double setpoint) {

        if(Math.abs(getPivotAbsEncoder() - setpoint) > PivotConstants.deadbandAngle) {
            if(getPivotAbsEncoder() > setpoint) {
                setPivotMotorNoBounds(0.4 * Math.abs(Math.sin(getPivotAbsEncoder() - setpoint)));
            } else if(setpoint > getPivotAbsEncoder()) {
                setPivotMotorNoBounds(-0.4 * Math.abs(Math.sin(setpoint - getPivotAbsEncoder())));
            }
        }
    }

    //TEST Feedforward and PID controller for pivot
    public void pivotWithFeedforwardPID(double desAngle, double desVelocity, double desAccel, double desPosition) {
        //desAngle = radians
        //desVelocity = rad / sec
        //desAccel = rad / sec^2
        double temp =pivotFeedForward.calculate(desAngle, desVelocity, desAccel)
            + pivotPIDController.calculate(getPivotAbsEncoder(), desPosition);
        pivotMotorRight.setVoltage(-(temp));
        pivotMotorLeft.setVoltage(temp);


        // rightMotor.setVoltage(feedForward.calculate(rightVelocitySetpoint)
        //     + rightPID.calculate(rightEncoder.getRate(), rightVelocitySetpoint));
      }

}