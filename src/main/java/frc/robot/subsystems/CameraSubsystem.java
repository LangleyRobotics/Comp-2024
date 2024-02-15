package frc.robot.subsystems;

import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;


public class CameraSubsystem extends SubsystemBase{
  PhotonCamera camera;

    public CameraSubsystem() {
        // Change this to match the name of your camera
        camera = new PhotonCamera("photonvision");


    }
    
    @Override
    public void periodic() {
        var result = camera.getLatestResult();
        if(hasTargets(result)) {
            List<PhotonTrackedTarget> allTargets = result.getTargets();
            PhotonTrackedTarget target = result.getBestTarget();

            // Get information from target.
            double yaw = target.getYaw(); //theta
            double pitch = target.getPitch(); //phi
            double area = target.getArea();
            double skew = target.getSkew();
            Transform3d pose = target.getBestCameraToTarget();
            int targetID = target.getFiducialId();
        }

    }

    public PhotonPipelineResult getResult() {
        return camera.getLatestResult();
    }
    
    public boolean hasTargets(PhotonPipelineResult result) {
        // Check if the latest result has any targets.
        return result.hasTargets();
    }

    //Returns the distance from camera to april tag in meters
    public double getDistanceToAprilTag() {
        PhotonPipelineResult result = getResult();
        if (result.hasTargets()) {
            // First calculate range
            double range =
                    PhotonUtils.calculateDistanceToTargetMeters(
                            CameraConstants.camHeight,
                            CameraConstants.targetHeight,
                            CameraConstants.camPitch,
                            Math.toRadians(result.getBestTarget().getPitch()));

            // Use this range as the measurement we give to the PID controller.
            // -1.0 required to ensure positive PID controller effort _increases_ range
            //**forwardSpeed = -controller.calculate(range, GOAL_RANGE_METERS);**
            return range;
        }
        return 0;
    }

    public double getPitch() {
        var result = camera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
        return target.getPitch();
    }

    public double getYaw() {
        var result = camera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
        return target.getYaw();
    }
}
