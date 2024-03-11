// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;

import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.MathMethods;
import frc.robot.Robot;
import frc.robot.Trajectories;


public class DriveSubsystem extends SubsystemBase {
  ShuffleboardTab gyrox = Shuffleboard.getTab("Navx");
  // Robot swerve modules
  private final SwerveModule frontLeft =
      new SwerveModule(
          DriveConstants.kFrontLeftDriveMotorPort,
          DriveConstants.kFrontLeftTurningMotorPort,
          DriveConstants.kFrontLeftDriveMotorReversed,
          DriveConstants.kFrontLeftTurningMotorReversed,
          DriveConstants.kFrontLeftAbsEncoderPort,
          DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
          DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
          true);

  private final SwerveModule rearLeft =
      new SwerveModule(
        DriveConstants.kRearLeftDriveMotorPort,
        DriveConstants.kRearLeftTurningMotorPort,
        DriveConstants.kRearLeftDriveMotorReversed,
        DriveConstants.kRearLeftTurningMotorReversed,
        DriveConstants.kRearLeftAbsEncoderPort,
        DriveConstants.kRearLeftDriveAbsoluteEncoderReversed,
        DriveConstants.kRearLeftDriveAbsoluteEncoderOffsetRad,
        true);

  private final SwerveModule frontRight =
      new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kFrontRightDriveMotorReversed,
        DriveConstants.kFrontRightTurningMotorReversed,
        DriveConstants.kFrontRightAbsEncoderPort,
        DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
        true);

  private final SwerveModule rearRight =
      new SwerveModule(
        DriveConstants.kRearRightDriveMotorPort,
        DriveConstants.kRearRightTurningMotorPort,
        DriveConstants.kRearRightDriveMotorReversed,
        DriveConstants.kRearRightTurningMotorReversed,
        DriveConstants.kRearRightAbsEncoderPort,
        DriveConstants.kRearRightDriveAbsoluteEncoderReversed,
        DriveConstants.kRearRightDriveAbsoluteEncoderOffsetRad,
        true);

  // The gyro sensor
  public final AHRS m_gyro = new AHRS(SPI.Port.kMXP, (byte) 200);
  

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
          },
          new Pose2d(0.0, 0.0, new Rotation2d()));
  

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    new Thread( () -> {
      try {
        new WaitCommand(1.0);
        zeroHeading(); //Didn't work??
        //initModulesReset(getGyroConnected());
        //m_gyro.setAngleAdjustment(180);
      } catch (Exception e) {
      }
    }).start();


    // Configure AutoBuilder last
      AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
          new PIDConstants(0.5, 0.0, 0.0), // Translation PID constants (original was (5.0, 0.0, 0.0))
          new PIDConstants(0.04, 0.0, 0.0), // Rotation PID constants (original was (5.0, 0.0, 0.0))
          Constants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s (original was 4.5)
          DriveConstants.driveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
          new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
      );
    
  }

  public void setPose(Pose2d aprilPose2d) {
    m_odometry.resetPosition(m_gyro.getRotation2d(),         
      new SwerveModulePosition[] {
        frontRight.getPosition(),
        frontLeft.getPosition(),
        rearRight.getPosition(),
        rearLeft.getPosition()
      }, aprilPose2d);
  }

  
  //Command Factory In Subsystem Test
  public SequentialCommandGroup AutoCommandFactory(Trajectory path) {
    var thetaController =
    new ProfiledPIDController(
      AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);
      
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      path,
      this::getPose, // Functional interface to feed supplier
      DriveConstants.kDriveKinematics,
      // Position controllers
      new PIDController(AutoConstants.kPXController, 0, 0),
      new PIDController(AutoConstants.kPYController, 0, 0),
      thetaController,
      this::setModuleStates,
      this);

      SequentialCommandGroup SequentialCommandOutput = new SequentialCommandGroup(new InstantCommand(() -> resetOdometry(path.getInitialPose())),
                                                                                    swerveControllerCommand,
                                                                                    new InstantCommand(() -> stopModules()));
      System.out.println(path.getInitialPose());
      return SequentialCommandOutput;
  }

  public SequentialCommandGroup AutoCommandFactory(Trajectory path, Boolean isPathPlanner, Pose2d startPose) {
    var thetaController =
    new ProfiledPIDController(
      AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);
      
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      path,
      this::getPose, // Functional interface to feed supplier
      DriveConstants.kDriveKinematics,
      // Position controllers
      new PIDController(AutoConstants.kPXController, 0, 0),
      new PIDController(AutoConstants.kPYController, 0, 0),
      thetaController,
      this::setModuleStates,
      this);

      SequentialCommandGroup SequentialCommandOutput = new SequentialCommandGroup(new InstantCommand(() -> resetOdometry(startPose)),
                                                                                    swerveControllerCommand,
                                                                                    new InstantCommand(() -> stopModules()));
      System.out.println(path.getInitialPose());
      return SequentialCommandOutput;
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  public Rotation2d getRotation2d() {
    // Rotation2d temp=new Rotation2d(m_gyro.getRotation2d().getDegrees()+48.5);
    //   return temp;
    return m_gyro.getRotation2d();
  }

  public double getRoll() {
    return (double) (m_gyro.getRoll());
  }

  public double getPitch() {
    return (double) (m_gyro.getPitch());
  }

  public boolean getGyroConnected() {
    return m_gyro.isConnected();
  }


  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          rearLeft.getPosition(),
          rearRight.getPosition()
        });


    //OUTPUT RELEVANT VALUES TO SMARTDASHBOARD
    SmartDashboard.putNumber("Robot Heading", getHeading());
    SmartDashboard.putBoolean("NavX connected?", getGyroConnected());

    SmartDashboard.putNumber("Absolute Encoder Angle Front Left", (Math.toDegrees(frontLeft.getAbsoluteEncoderRad())));
    SmartDashboard.putNumber("Current Angle Front Left", (Math.toDegrees(frontLeft.getTurningPosition())));
    SmartDashboard.putNumber("Front Left Offset", Math.toDegrees(frontLeft.getAbsoluteEncoderRad()) - Math.toDegrees(frontLeft.getTurningPosition()));

    SmartDashboard.putNumber("Absolute Encoder Angle Front Right", Math.toDegrees(frontRight.getAbsoluteEncoderRad()));
    SmartDashboard.putNumber("Current Angle Front Right", Math.toDegrees(frontRight.getTurningPosition()));
    SmartDashboard.putNumber("Front Right Offset", Math.toDegrees(frontRight.getAbsoluteEncoderRad()) - Math.toDegrees(frontRight.getTurningPosition()));


    SmartDashboard.putNumber("Absolute Encoder Angle Rear Left", Math.toDegrees(rearLeft.getAbsoluteEncoderRad()));
    SmartDashboard.putNumber("Current Angle Rear Left", ((Math.toDegrees(rearLeft.getTurningPosition()))));
    SmartDashboard.putNumber("Rear Left Offset", Math.toDegrees(rearLeft.getAbsoluteEncoderRad()) - Math.toDegrees(rearLeft.getTurningPosition()));


    SmartDashboard.putNumber("Absolute Encoder Angle Rear Right", Math.toDegrees(rearRight.getAbsoluteEncoderRad()));
    SmartDashboard.putNumber("Current Angle Rear Right", ((Math.toDegrees(rearRight.getTurningPosition()))));
    SmartDashboard.putNumber("Rear Right Offset", Math.toDegrees(rearRight.getAbsoluteEncoderRad()) - Math.toDegrees(rearRight.getTurningPosition()));

    //ROBOT CONTROLLER VALUES
    
    SmartDashboard.putBoolean("Roborio Status", RobotController.isBrownedOut());
    SmartDashboard.putNumber("Brownout Voltage", RobotController.getBrownoutVoltage());
    SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Match Time FPGA", RobotController.getFPGATime());
    


    /* 
    SmartDashboard.putBoolean("Front Left Turning Encoder inverted: ", frontLeft.getTurningEncInvert());
    SmartDashboard.putBoolean("Front Right Turning Encoder inverted: ", frontRight.getTurningEncInvert());
    SmartDashboard.putBoolean("Rear Left Turning Encoder inverted: ", rearLeft.getTurningEncInvert());
    SmartDashboard.putBoolean("Rear Right Turning Encoder inverted: ", rearRight.getTurningEncInvert());
    */
    SmartDashboard.putBoolean("Front Left Turning Motor inverted: ", frontLeft.getTurningMotorInvert());
    SmartDashboard.putBoolean("Front Right Turning Motor inverted: ", frontRight.getTurningMotorInvert());
    SmartDashboard.putBoolean("Rear Left Turning Motor inverted: ", rearLeft.getTurningMotorInvert());
    SmartDashboard.putBoolean("Rear Right Turning Motor inverted: ", rearRight.getTurningMotorInvert());

    SmartDashboard.putNumber("Current X Pose", getPose().getX());
    SmartDashboard.putNumber("Current Y Pose", getPose().getY());
    SmartDashboard.putNumber("Current Rot2D Pose", getPose().getRotation().getDegrees());

  
    
    SmartDashboard.putNumber("Current Pitch", getPitch());
    SmartDashboard.putNumber("Current Roll", getRoll());

    SmartDashboard.putNumberArray("Advantage Scope Swerve States", new double[] 
    {Math.toDegrees(frontLeft.getTurningPosition()), frontLeft.getDriveVelocity(),
      Math.toDegrees(frontRight.getTurningPosition()), frontRight.getDriveVelocity(),
      Math.toDegrees(rearLeft.getTurningPosition()), rearLeft.getDriveVelocity(),
      Math.toDegrees(rearRight.getTurningPosition()), rearRight.getDriveVelocity()});

    SmartDashboard.putNumberArray("Advantage Scope Swerve Desired States", new double[] 
    {
      SmartDashboard.getNumber(DriveConstants.kFrontLeftAbsEncoderPort + "desired speed", 0),
      SmartDashboard.getNumber(DriveConstants.kFrontLeftAbsEncoderPort + "desired rotation", 0),
      SmartDashboard.getNumber(DriveConstants.kFrontRightAbsEncoderPort + "desired speed", 0),
      SmartDashboard.getNumber(DriveConstants.kFrontRightAbsEncoderPort + "desired rotation", 0),
      SmartDashboard.getNumber(DriveConstants.kRearLeftAbsEncoderPort + "desired speed", 0),
      SmartDashboard.getNumber(DriveConstants.kRearLeftAbsEncoderPort + "desired rotation", 0),
      SmartDashboard.getNumber(DriveConstants.kRearRightAbsEncoderPort + "desired speed", 0),
      SmartDashboard.getNumber(DriveConstants.kRearRightAbsEncoderPort + "desired rotation", 30)
    });

  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

 
  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          rearLeft.getPosition(),
          rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.kMaxSpeedMetersPerSecond);
    // frontLeft.setDesiredState(swerveModuleStates[0]);
    // frontRight.setDesiredState(swerveModuleStates[1]);
    // rearLeft.setDesiredState(swerveModuleStates[2]);
    // rearRight.setDesiredState(swerveModuleStates[3]);

    frontRight.setDesiredState(swerveModuleStates[0]);
    frontLeft.setDesiredState(swerveModuleStates[1]);
    rearRight.setDesiredState(swerveModuleStates[2]);
    rearLeft.setDesiredState(swerveModuleStates[3]);
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    rearLeft.stop();
    rearRight.stop();
  }
  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Constants.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeft.resetEncoders();
    rearLeft.resetEncoders();
    frontRight.resetEncoders();
    rearRight.resetEncoders();
  }


  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  //TEST Realign wheels at the start of each match
  public void initModulesReset() {
    double frontRightDesAngle = DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad;
    SwerveModuleState frontRightDesState = new SwerveModuleState(0.001, new Rotation2d(frontRightDesAngle));
    frontRight.resetInitModule(frontRightDesState, frontRightDesAngle);

    double rearRightDesAngle = DriveConstants.kRearRightDriveAbsoluteEncoderOffsetRad;
    SwerveModuleState rearRightDesState = new SwerveModuleState(0.001, new Rotation2d(rearRightDesAngle));
    rearRight.resetInitModule(rearRightDesState, rearRightDesAngle);

    double rearLeftDesAngle = DriveConstants.kRearLeftDriveAbsoluteEncoderOffsetRad;
    SwerveModuleState rearLeftDesState = new SwerveModuleState(0.001, new Rotation2d(rearLeftDesAngle));
    rearLeft.resetInitModule(rearLeftDesState, rearLeftDesAngle);

    double frontLeftDesAngle = DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad;
    SwerveModuleState frontLeftDesState = new SwerveModuleState(0.001, new Rotation2d(frontLeftDesAngle));
    frontLeft.resetInitModule(frontLeftDesState, frontLeftDesAngle);

    System.out.println("Init Module Reset Test");
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return new ChassisSpeeds(m_gyro.getVelocityX(), m_gyro.getVelocityY(), Math.toRadians(m_gyro.getRate()));
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(moduleStates);
  }
}