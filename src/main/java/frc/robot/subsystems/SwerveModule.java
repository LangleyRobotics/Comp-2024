// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.MathMethods;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import com.ctre.phoenix6.hardware.CANcoder;



public class SwerveModule {


  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turningEncoder;


  private final PIDController m_drivePIDController =
      new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  private final CANcoder absoluteEncoder;
  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffsetRad;

  private final boolean relEncoderInverted;
  

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          ModuleConstants.kPModuleTurningController,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  private final PIDController turningPidController;
  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel The channel of the drive motor.
   * @param turningMotorChannel The channel of the turning motor.
   * @param driveMotorReversed Whether the drive encoder is reversed.
   * @param turningMotorReversed Whether the turning encoder is reversed.
   * @param absoluteEncoderChannel The channel of the absolute encoder
   * @param absoluteEncoderReversed Whether the absolute encoder is reversed
   * @param absoluteEncoderOffsetRad The offset (radians) of absolute encoder
   * @param relEncoderInverted
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      boolean driveMotorReversed,
      boolean turningMotorReversed,
      int absoluteEncoderChannel,
      boolean absoluteEncoderReversed,
      double absoluteEncoderOffsetRad,
      boolean relEncoderInverted) {


    this.absoluteEncoderOffsetRad = absoluteEncoderOffsetRad;
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    
    this.relEncoderInverted = relEncoderInverted;

    driveMotor = new CANSparkMax(driveMotorChannel,MotorType.kBrushless);
    turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

    driveMotor.setInverted(!driveMotorReversed);
    turningMotor.setInverted(turningMotorReversed);
    

    driveEncoder = driveMotor.getEncoder();

    
    turningEncoder = turningMotor.getEncoder();


    absoluteEncoder = new CANcoder(absoluteEncoderChannel);
    
    turningMotor.setInverted(!relEncoderInverted);
    
    //CHANGE THESE CONSTANTS
    driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
    turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);


  

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoders();
  }


  public double getDrivePosition() {
      return driveEncoder.getPosition();
  }

  public double getTurningPosition() {
      return MathMethods.moduloAngle((turningEncoder.getPosition()));
  }

  public double getDriveVelocity() {
      return driveEncoder.getVelocity();
  }

  public double getTurningVelocity() {
      return turningEncoder.getVelocity();
  }

  public boolean getTurningMotorInvert() {
    return turningMotor.getInverted();
  }
  
  public boolean getTurningEncInvert() {
      return turningEncoder.getInverted();
  }
  

  public double getAbsoluteEncoderRad() {
    double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    angle = Math.toRadians(angle);
    angle -= absoluteEncoderOffsetRad;
    return MathMethods.moduloAngle((angle * (absoluteEncoderReversed ? -1.0 : 1.0)));
  }

  /** Resets all the SwerveModule encoders. */
  public void resetEncoders() {
    driveEncoder.setPosition(0);
    turningEncoder.setPosition(getAbsoluteEncoderRad());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
          getDrivePosition(), new Rotation2d(getTurningPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
        stop();
        return;
    }
    state = SwerveModuleState.optimize(state, getState().angle);
    driveMotor.set(state.speedMetersPerSecond / Constants.kMaxSpeedMetersPerSecond);
    turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "]  state", state.toString());
    SmartDashboard.putNumber(absoluteEncoder.getDeviceID() + "desired speed", state.speedMetersPerSecond);
    SmartDashboard.putNumber(absoluteEncoder.getDeviceID() + "desired rotation", state.angle.getDegrees());
  }
  public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
    
  }

  public void resetInitModule(SwerveModuleState desState, double desAngle) {
    while(Math.abs(getAbsoluteEncoderRad() - desAngle) > 5) {
      setDesiredState(desState);
    }
  }
}