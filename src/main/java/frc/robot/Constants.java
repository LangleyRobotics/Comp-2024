// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
 
  public static final double kMaxSpeedMetersPerSecond = 4;
  public static final double kMaxAccelerationMetersPerSecondSquared = 2;
  public static final double kMaxAngularSpeedRadiansPerSecond = MathMethods.Tau;
  public static final double kMaxAngularAccelerationRadiansPerSecondSquared = MathMethods.Tau;
  public static final CustomHolonomicDrive holonomicDrive = new CustomHolonomicDrive(
                                                              new PIDController(0.5, 0.0, 0.0),
                                                              new PIDController(0.04, 0, 0));

  public static final class LEDConstants {
    public static final int kAddressableLightsID = 0;
    public static final int kAddressableLightsLength = 30;
    public static final int wordDisplayLength = 15;

  }

  public static final class DriveConstants {
    //CAN IDs for Sparkmaxes
    public static final int kFrontRightTurningMotorPort = 1;
    public static final int kRearRightTurningMotorPort = 3;
    public static final int kRearLeftTurningMotorPort = 5;
    public static final int kFrontLeftTurningMotorPort = 7;

    public static final int kFrontRightDriveMotorPort = 2;
    public static final int kRearRightDriveMotorPort = 4;
    public static final int kRearLeftDriveMotorPort = 6;
    public static final int kFrontLeftDriveMotorPort = 8;


    //Before = all true
    public static final boolean kFrontRightTurningMotorReversed = true;
    public static final boolean kRearRightTurningMotorReversed = true;
    public static final boolean kRearLeftTurningMotorReversed = true;
    public static final boolean kFrontLeftTurningMotorReversed = true;

    public static final boolean kFrontRightDriveMotorReversed = true;
    public static final boolean kRearRightDriveMotorReversed = false;
    public static final boolean kRearLeftDriveMotorReversed = true;
    public static final boolean kFrontLeftDriveMotorReversed = true;

    //CAN IDs for Encoders on Swerve Modules
    public static final int kFrontRightAbsEncoderPort = 16;
    public static final int kRearRightAbsEncoderPort = 17;
    public static final int kRearLeftAbsEncoderPort = 18;
    public static final int kFrontLeftAbsEncoderPort = 19;
    
    //Before = all false
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final boolean kRearRightDriveAbsoluteEncoderReversed = false;
    public static final boolean kRearLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;

    //Fix Front and rear left offsets (radians)
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(0);//0.04418;//174
    public static final double kRearRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(180);//0.4504;//5
    public static final double kRearLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(0);//0.1489;//-10
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(0);//0.1992;//-4

    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = 0.533;

    // Distance between front and back wheels on robot
    public static final double kWheelBase = 0.533;

    //Distance between center of robot and furthest module
    public static final double driveBaseRadius = 0.5 * (kTrackWidth / Math.sin(Math.atan(kTrackWidth / kWheelBase)));

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //+- = Front Right 
            new Translation2d(kWheelBase / 2, kTrackWidth / 2), //++ = Front Left 
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), //-- = Rear Right 
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2)); //-+ = Rear Left


    public static final boolean kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    // These values "should" be right dw
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 0.8;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;

    public static final double kSlowDriveCoefficient = 0.15;

  }

  //ID Numbers for Xbox controller buttons
  public static final class Buttons {
    public static final int A = 1;
    public static final int B = 2;
    public static final int X = 3;
    public static final int Y = 4;
    public static final int LB = 5;
    public static final int RB = 6;

    //That one button next to the menu button
    public static final int Maria = 7;
    public static final int Menu = 8;

    public static final int L3 = 9;
    public static final int R3 = 10;

    public static final int UP_ARR = 0;
    public static final int RIGHT_ARR = 90;
    public static final int DOWN_ARR = 180;
    public static final int LEFT_ARR = 270;

    
  }
  
    public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = Math.PI;
    //kSteerReduction = (7.0 / 150.0) for Mk4i, (1.0 / 12.8) for Mk4
    public static final double kSteerReduction = (7.0 / 150.0);
    public static final double kDriveReduction = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0);
    public static final int kEncoderCPR = 4096;
    public static final double kPTurning = 0.5;
    public static final double kDriveEncoderRot2Meter = kDriveReduction * Math.PI * kWheelDiameterMeters * (89.0/100.0);
    public static final double kTurningEncoderRot2Rad = kSteerReduction * MathMethods.Tau;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI * kDriveReduction) / (double) kEncoderCPR;

    public static final double kTurningEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI * kSteerReduction) / (double) kEncoderCPR;

    public static final double kPModuleTurningController = 1;

    public static final double kPModuleDriveController = 1;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kSecondaryControllerPort = 1;

    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 4;
    public static final int kDriverFieldOrientedButtonIdx = 1;
    public static final double kDeadband = 0.2;
  }

  public static final class IntakeConstants {
    public static final int kIntakeMotor = 10;
    public static final double kIntakeMotorSpeed = 0.65; //value from 0-1
  }

  public static final class ShooterConstants {
    //Sparkmax ports
    public static final int kShooterMotorPort = 9;
    //Speed percentage (scale of 0-1)
    public static final double kShooterMotorSpeed = 1.0;
  }

  public static final class TrapConstants {
    public static final  int kTrapMotorPort = 15;
    public static final  double kTrapSpeed = 0.3;
  }
  public static final class PivotConstants {
    public static final int kPivotMotorRight = 11;
    public static final int kPivotMotorLeft = 12;
    public static final double kPivotMotorSpeed = 1.0; //value from 0-1
    public static final double kPivotMotorAccel = 0.5;

    //Pivot Positions
    public static final double kMaxPivotPosition = 180;
    public static final double kMinPivotPosition = 75;
    public static final double kAmpPosition = 90;
    public static final double shootUpClosePosition = 170; //position of pivot encoder for shooting when up against the speaker
    public static final double shootSideRingsPosition = 170;

    public static final double kPivotOffset = 181;
    public static final double disPerRot = 360;
    public static final double pivotCompensation = 0.08;
    public static final double kPivotEncoderBreakpoint = 0.5;

    public static final double kS_Pivot = 0.3;
    public static final double kG_Pivot = 1.75;
    public static final double kV_Pivot = 1.95;
    public static final double kA_Pivot = 0.4;

    public static final double kP_Pivot = 0.15;
    public static final double kI_Pivot = 0.00;
    public static final double kD_Pivot = 0.02;

    public static final double kAprilCamSpeedFactor = 0.05;
    public static final double kAprilCamMaxSpeedMetersPerSecond = 0.69;
    public static final double kAprilCamDeadbandDegrees = 3;
    public static final double kAprilCamMinSpeed = 0.13;

    //TEST Simple goToSetpoint() method constants
    public static final double tinyPivotSpeed = 0.7;
    public static final double tinyPivotAccel = 0.05;
    public static final double deadbandAngle = 3;

    public static final double pivotSetpointFactor = 1;
  }

  public static final class ClimbConstants {
    public static final int kClimbMotorRight = 13;
    public static final int kClimbMotorLeft = 14;
    public static final double kClimbMotorSpeed = 0.6; //value from 0-1

    //through bore encoder values
    public static final double rightUpperLimit = 100;
    public static final double rightLowerLimit = -100;

    public static final double leftUpperLimit = 100;
    public static final double leftLowerLimit = -100;

    //Preferences constants
    public static final String rightPosKey = "Right Key";
    public static final String leftPosKey = "Left Key";

    public static final double disPerRot = 1;
  }
  
  public static final class CameraConstants {
    //Meters
    public static final double h1 = 1.43; //AprilTag to camera height
    public static final double h2 = 0.47; //goal to AprilTag height
    public static final double L = 0.66; //shooter length
    public static final double tempTheta = Math.PI / 4; //TEST just to maybe figure out setpoint
    public static final double deltaX = 0.08; //camera to pivot distance

    public static final double camHeight = 0.08;
    public static final double targetHeight = h1 + camHeight;

    //Radians
    public static final double camPitch = 0;

    //Apriltag Megatag Botpose array positions
    public static final int kMegaBotPoseTransX = 0;
    public static final int kMegaBotPoseTransY = 1;
    public static final int kMegaBotPoseTransZ = 2;
    public static final int kMegaBotPoseRoll = 3;
    public static final int kMegaBotPosePitch = 4;
    public static final int kMegaBotPoseYaw = 5;
  }

  public static final class LimelightConstants {
    public static final double klimelightOneHeight = 0.0;
    public static final double klimelightOneAngleDeg = 0.0;

    public static final int kledModePipeline = 0;
    public static final int kledModeOff = 1;
    public static final int kledModeBlink = 2;
    public static final int kledModeOn = 3;

    public static final int kcamModeVisionProcessor = 0;
    public static final int kcamModeDriverCamera = 1;

    public static final int kpipelineZero = 0;
    public static final int kpipelineOne = 1;

    public static final double klimelightTwoHeight = 0.0;
    public static final double klimelightTwoAngleDeg = 0.0;

    //Apriltag Megatag Botpose array positions
    public static final int kMegaBotPoseTransX = 0;
    public static final int kMegaBotPoseTransY = 1;
    public static final int kMegaBotPoseTransZ = 2;
    public static final int kMegaBotPoseRoll = 3;
    public static final int kMegaBotPosePitch = 4;
    public static final int kMegaBotPoseYaw = 5;
  }

  public static final class AutoConstants {

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;
    public static final double kAutoMaxSpeedMetersPerSecond = 3;
    public static final double kAutoMaxAccelerationMetersPerSecondSquared = 2;
    public static final double kAprilDriveSpeedFactor = 0.5;
    public static final double kAprilDriveMaxSpeedMetersPerSecond = 0.69;
    public static final double kAprilDriveDeadbandDegrees = 3;
    public static final double kAprilDriveMinSpeed = 0.13;
    public static final double kFieldEndXCoordinate = 16.5;

    public static final double kLameSpeedCap = 1.0;
    public static final double kLameAccelCap = 1.0;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);
  }
  
}
