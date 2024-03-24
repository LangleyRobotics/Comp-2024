package frc.robot.commands;


import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class OTFTrajectoryFactory {


    private final DriveSubsystem swerveSubsystem;
    private final List<Translation2d> listOfTranslations;
    private final Pose2d endPose;
    private final TrajectoryConfig config;
    private Supplier<Pose2d> currentPose;

    public OTFTrajectoryFactory(DriveSubsystem swerveSubsystem, Supplier<Pose2d> currentPose, List<Translation2d> listOfTranslations, Pose2d endPose, TrajectoryConfig config) {
        this.swerveSubsystem = swerveSubsystem;
        this.listOfTranslations = listOfTranslations;
        this.endPose = endPose;
        this.config = config;
        this.currentPose = currentPose;
    }
    
    public SequentialCommandGroup OTFTrajectoryGet() {
        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);

        Trajectory path = TrajectoryGenerator.generateTrajectory(currentPose.get(), listOfTranslations, endPose, config);
        // System.out.println(currentPose.get());

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            path,
            swerveSubsystem::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            swerveSubsystem::setModuleStates,
            swerveSubsystem);
        
        return new SequentialCommandGroup(new InstantCommand(() -> swerveSubsystem.resetOdometry(path.getInitialPose())), swerveControllerCommand, new InstantCommand(() -> swerveSubsystem.stopModules()));

    }
        
    

}
