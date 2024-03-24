package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.CustomHolonomicDrive;

public class DriveToPointCmd extends Command {
    DriveSubsystem m_swerveSubsystem;
    Supplier<Pose2d> m_targetPoseSupplier;
    double m_maxSpeedWanted;
    CustomHolonomicDrive m_HoloDrive;
    Supplier<Double> xSpeed;
    Supplier<Double> ySpeed;
    Supplier<Double> zRotation;

    public DriveToPointCmd(DriveSubsystem swerveSubsystem, Supplier<Pose2d> targetPoseSupplier,
      CustomHolonomicDrive HoloDrive) {
        m_swerveSubsystem = swerveSubsystem;
        m_targetPoseSupplier = targetPoseSupplier;
        m_HoloDrive = HoloDrive;

        addRequirements(swerveSubsystem);
      }


    @Override
    public void initialize() {

    }
  
    @Override
    public void execute() {

        ChassisSpeeds speeds = m_HoloDrive.calculate(m_swerveSubsystem.getPose(), m_targetPoseSupplier.get());
        m_swerveSubsystem.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
    
    }
  
    @Override
    public void end(boolean interrupted) {
          //m_swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
      return m_HoloDrive.targetPoseAchieved();
        //return false;
    }

    
}
