package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.CameraSubsystem;

import frc.robot.Constants;
import frc.robot.MathMethods;

public class SwerveControllerCmd extends Command {

    private final DriveSubsystem swerveSubsystem;
    private final CameraSubsystem cameraSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private final Supplier<Boolean> finishedCondition;

    public SwerveControllerCmd(DriveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction, Supplier<Boolean> finishedCondition) {
        this.swerveSubsystem = swerveSubsystem;
        this.cameraSubsystem = null;
        this.finishedCondition = finishedCondition;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(Constants.kMaxAccelerationMetersPerSecondSquared);
        this.yLimiter = new SlewRateLimiter(Constants.kMaxAccelerationMetersPerSecondSquared);
        this.turningLimiter = new SlewRateLimiter(Constants.kMaxAngularAccelerationRadiansPerSecondSquared);
        addRequirements(swerveSubsystem);
    }

    public SwerveControllerCmd(DriveSubsystem swerveSubsystem, CameraSubsystem cameraSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.cameraSubsystem = cameraSubsystem;
        this.xSpdFunction = () -> (MathMethods.speedMax(AutoConstants.kAprilDriveSpeedFactor*Math.sin(Math.toRadians(cameraSubsystem.getPitch())),
            AutoConstants.kAprilDriveMaxSpeedMetersPerSecond, AutoConstants.kAprilDriveDeadbandDegrees, AutoConstants.kAprilDriveMinSpeed, () -> cameraSubsystem.getPitch()));
        
        this.ySpdFunction = () -> (-MathMethods.speedMax(AutoConstants.kAprilDriveSpeedFactor*Math.sin(Math.toRadians(cameraSubsystem.getYaw())), 
            AutoConstants.kAprilDriveMaxSpeedMetersPerSecond, AutoConstants.kAprilDriveDeadbandDegrees, AutoConstants.kAprilDriveMinSpeed, () -> cameraSubsystem.getYaw()));
            
        this.turningSpdFunction = () -> (0.0);
        this.fieldOrientedFunction = () -> false;
        
        this.xLimiter = new SlewRateLimiter(Constants.kMaxAccelerationMetersPerSecondSquared);
        this.yLimiter = new SlewRateLimiter(Constants.kMaxAccelerationMetersPerSecondSquared);
        this.turningLimiter = new SlewRateLimiter(Constants.kMaxAngularAccelerationRadiansPerSecondSquared);

        this.finishedCondition  = () -> false;

        addRequirements(swerveSubsystem, cameraSubsystem);
    }

    @Override
    public void initialize() {
    }


    @Override
    public void execute() {

        double ySpeed = -ySpdFunction.get();
        double xSpeed = xSpdFunction.get();
        //I PUT A NEGATIVE ON THE TURNING SPEED AAAHHHHH
        double turningSpeed = -1 * turningSpdFunction.get();

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * Constants.kMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * Constants.kMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * Constants.kMaxAngularSpeedRadiansPerSecond;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}