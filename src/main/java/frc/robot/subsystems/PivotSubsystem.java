package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.apriltag.AprilTag;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.MathMethods;

public class PivotSubsystem extends SubsystemBase {
    private final CANSparkMax pivotMotorRight = new CANSparkMax(PivotConstants.kPivotMotorRight, MotorType.kBrushless);
    private final CANSparkMax pivotMotorLeft = new CANSparkMax(PivotConstants.kPivotMotorLeft, MotorType.kBrushless);
    private final DutyCycleEncoder pivotAbsEncoder = new DutyCycleEncoder(0);

    private final ArmFeedforward pivotFeedForward = new ArmFeedforward(PivotConstants.kS_Pivot, PivotConstants.kG_Pivot, PivotConstants.kV_Pivot, PivotConstants.kA_Pivot);
    private final PIDController pivotPIDController = new PIDController(PivotConstants.kP_Pivot, PivotConstants.kI_Pivot, PivotConstants.kD_Pivot);

    double absEncoderRaw = 0;
    double offset = 0;
    

    public PivotSubsystem() {
        // Configures the encoder to return a distance for every rotation
        pivotAbsEncoder.setDistancePerRotation(PivotConstants.disPerRot);
        offset = pivotAbsEncoder.getDistance() - PivotConstants.kPivotOffset;
        pivotPIDController.setTolerance(PivotConstants.deadbandAngle);
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Encoder", getPivotAbsEncoder());
        SmartDashboard.putBoolean("Pivot Encoder Connected?", pivotAbsEncoder.isConnected());   
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

    //reset the pivot encoder to the be 0-180 at illegal position
    public void resetPivotEncoder() {
        offset = pivotAbsEncoder.getDistance() - PivotConstants.kPivotOffset + 183;
    }

    //Send arm to a specific setpoint
    public void goToSetpoint(double desPosition) {
        //desPosition = radians
        double output = pivotPIDController.calculate(getPivotAbsEncoder(), desPosition);
        if(output > 0 && getPivotAbsEncoder() >= 155) { 
            System.out.println("Forward limiting");
            // pivot going forward, slow so doesnt hit ground
            pivotMotorRight.setVoltage(-0.4);
            pivotMotorLeft.setVoltage(0.4);
        } else if(desPosition < PivotConstants.kAmpPosition && output < 0) { 
            System.out.println("Backwards limiting");

            // pivot going backwards, slow so doesnt hit ground
            setPivotMotorNoBounds(MathMethods.signDouble(Math.cos(getPivotAbsEncoder()))*0.02 - PivotConstants.pivotCompensation * Math.cos(Math.toRadians(getPivotAbsEncoder())));
        } else if(Math.abs(getPivotAbsEncoder() - desPosition) > PivotConstants.deadbandAngle){
            // pivot is freeeee
            System.out.println("Pivot is Free");
            pivotMotorRight.setVoltage(-output);
            pivotMotorLeft.setVoltage(output);
        }
        System.out.println("output: " + output);
        System.out.println("actual, target " + getPivotAbsEncoder() + ", " + desPosition);
    }

    public boolean isAtSetpoint() {
        return pivotPIDController.atSetpoint();
    }

    //TEST Feedforward and PID controller for pivot
    public void pivotWithPID(double desPosition) {
        //desPosition = radians
        double output = pivotPIDController.calculate(getPivotAbsEncoder(), desPosition);
        // System.out.println(pivotMotorLeft.get());
        if(output > 0 && desPosition >= 160) {
            pivotMotorRight.setVoltage(-0.03);
            pivotMotorLeft.setVoltage(0.03);
        } else if(desPosition < 91 && output < 0) {
            setPivotMotorNoBounds(MathMethods.signDouble(Math.cos(getPivotAbsEncoder()))*0.02 - PivotConstants.pivotCompensation * Math.cos(Math.toRadians(getPivotAbsEncoder())));
        } else if (output > 0) {
            pivotMotorRight.setVoltage(-1*output);
            pivotMotorLeft.setVoltage(1*output);
        } else if(output < 0) {
            pivotMotorRight.setVoltage(-4*output);
            pivotMotorLeft.setVoltage(4*output);
        }
      }

}