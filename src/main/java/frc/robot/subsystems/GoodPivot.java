// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import java.util.function.Supplier;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.PivotConstants;
import frc.robot.ShooterLookupTable;
import frc.robot.Constants.PivotConstants;


public class GoodPivot extends ProfiledPIDSubsystem {
  /** Creates a new GoodPivot. */
  private final CANSparkMax pivotMotorRight = new CANSparkMax(PivotConstants.kPivotMotorRight, MotorType.kBrushless);
  private final CANSparkMax pivotMotorLeft = new CANSparkMax(PivotConstants.kPivotMotorLeft, MotorType.kBrushless);
  private final DutyCycleEncoder pivotAbsEncoder = new DutyCycleEncoder(0);
  private SparkPIDController pivotPID;
  double absEncoderRaw = 0;
  double offset = 0;
  public GoodPivot() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            PivotConstants.kP_Pivot,
            PivotConstants.kI_Pivot,
            PivotConstants.kA_Pivot,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(100, 50)));
            pivotPID = pivotMotorLeft.getPIDController();
            pivotAbsEncoder.setDistancePerRotation(PivotConstants.disPerRot);
            offset = pivotAbsEncoder.getDistance() - PivotConstants.kPivotOffset;
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("Pivot Encoder", getPivotAbsEncoder());
    SmartDashboard.putBoolean("Pivot Encoder Connected?", pivotAbsEncoder.isConnected());
    SmartDashboard.putNumber("Pivot Right Voltage", pivotMotorRight.getBusVoltage());
    SmartDashboard.putNumber("Pivot Left Voltage", pivotMotorLeft.getBusVoltage());
  }

  public void setSpeed(double spd){
    pivotMotorLeft.set(spd);
    pivotMotorRight.set(spd);
  }

  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    double feedforward = 0;
    pivotMotorLeft.setVoltage(output + feedforward);
    pivotMotorRight.setVoltage(-(output + feedforward));
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getPivotAbsEncoder();
  }
  public void zeroPivot(){

  }
  public double getPivotAbsEncoder() {
    pivotAbsEncoder.setDistancePerRotation(PivotConstants.disPerRot);
    return pivotAbsEncoder.getDistance() - offset;
  }
  public void setPosition(double setpoint) {
    getController().setGoal(setpoint);
  }
  public Command updatePosition(Supplier<Double> setpoint)
  {
        return new RunCommand(() -> setPosition(setpoint.get()), this);
  }
  public boolean isAtGoal() {
    return getController().atGoal();
  }
  

}
