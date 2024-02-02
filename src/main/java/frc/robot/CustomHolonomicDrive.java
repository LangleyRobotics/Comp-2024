package frc.robot;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


//Custom holonomic drive controller
//PID controller for translation
//PID controller for rotation

public class CustomHolonomicDrive {
    private Pose2d poseError = new Pose2d();
    private Pose2d poseDeadband = new Pose2d(0.1, 0.1, new Rotation2d(Math.toRadians(0.8)));
    
 private final PIDController xyController;
    private final PIDController thetaController;


    public CustomHolonomicDrive(PIDController xyController, PIDController thetaController) {
        this.xyController = xyController;
        this.thetaController = thetaController;
        this.thetaController.enableContinuousInput(-180, 180);
    }

    public ChassisSpeeds calculate(Pose2d currentPose, Pose2d targetPose) {
        poseError = targetPose.relativeTo(currentPose);

        double xFeedback = this.xyController.calculate(currentPose.getX(), targetPose.getX());
        double yFeedback = this.xyController.calculate(currentPose.getY(), targetPose.getY());
        double thetaFeedback = this.thetaController.calculate(currentPose.getRotation().getDegrees(),
            targetPose.getRotation().getDegrees());

        double x = xFeedback;
        double y = yFeedback;

        return ChassisSpeeds.fromFieldRelativeSpeeds(x * Constants.kMaxSpeedMetersPerSecond,
                                                        y * Constants.kMaxSpeedMetersPerSecond,
                                                        thetaFeedback * 1.5,
                                                        currentPose.getRotation());
    }

    public boolean targetPoseAchieved() {
        final var translationError = poseError.getTranslation();
        final var translationDeadband = poseDeadband.getTranslation();
        final var rotationError = poseError.getRotation();
        final var rotationDeadband = poseDeadband.getRotation();
        return Math.abs(translationError.getX()) < translationDeadband.getX()
            && Math.abs(translationError.getY()) < translationDeadband.getY()
            && Math.abs(rotationError.getDegrees()) < rotationDeadband.getDegrees();
    }
    
}