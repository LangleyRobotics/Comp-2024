package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.PivotConstants;

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
    
}
