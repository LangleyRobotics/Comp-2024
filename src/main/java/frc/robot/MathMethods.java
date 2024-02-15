package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.CameraSubsystem;

public class MathMethods {
    public static final double Tau = 2*Math.PI;
    public boolean test = false;
    public static double moduloAngle(double angle) {
        while (angle > Math.PI) {
            angle -= Tau;
        }
        while (angle < -Math.PI) {
            angle += Tau;
        }

        return angle;
    }

    //Define acceleration paramaters for auto-level
    //Use  AutoConstants.kAutoBalanceMaxSpeedMetersPerSecond as accelCap to implement in balance funtion
    public static double speedMax(double initVel, double maxSpeed, double deadband, double minSpeed, Supplier<Double> angle) 
    {
        if (Math.abs(initVel) > maxSpeed) {
            return maxSpeed*((Math.abs(initVel))/initVel);
        } else if (Math.abs(angle.get()) < deadband) {
            return 0;
        }  else if (Math.abs(initVel)<minSpeed) {
            return minSpeed*((Math.abs(initVel))/initVel);
        } else {
            return initVel;
        }
    }

    public static Pose2d doubleArrToPose2d(double[] poseArr) {
        return new Pose2d(poseArr[0], poseArr[1], new Rotation2d(poseArr[5]));
    }
    
    public static double speedMax2(double initVel, double maxSpeed, double deadband) 
    {
        if (Math.abs(initVel) > maxSpeed) {
            return maxSpeed*((Math.abs(initVel))/initVel);
        } else if (Math.abs(initVel) < deadband) {
            return 0;
        } else {
            return initVel;
        }
    }

    public static int signDouble(double num) {
        return (int) (Math.abs(num)/num);
    }

    public static Pose2d poseXFlipper(Pose2d pose) {
        double rot = pose.getRotation().getRadians();
        return new Pose2d(AutoConstants.kFieldEndXCoordinate - pose.getX(), pose.getY(), new Rotation2d(signDouble(rot) * (Math.PI - Math.abs(rot))));
    }

    //TEST Calculate the setpoint needed to auto align the shooter with the goal
    //Returns the angle in radians
    public static double calculateSetpoint(CameraSubsystem cameraSubsystem) {
        double h1 = CameraConstants.h1;
        double h2 = CameraConstants.h2;
        double L = CameraConstants.L;
        double tempTheta = CameraConstants.tempTheta;
        double length1 = cameraSubsystem.getDistanceToAprilTag() * Math.cos(cameraSubsystem.getPitch());
        double length2 = length1 + CameraConstants.deltaX + L * Math.cos(tempTheta);
        double constant = (h1 + h2 - L * Math.sin(tempTheta)) / (length2);

        //Only works if the angle between shooter arm and shooter (alpha) = 45 degrees
        return Math.atan((1 - constant) / (1 + constant)) * PivotConstants.pivotSetpointFactor;
    }
    
}
