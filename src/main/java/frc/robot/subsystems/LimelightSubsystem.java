package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.MathMethods;


public class LimelightSubsystem extends SubsystemBase{
    private static LimelightSubsystem instance;
    private NetworkTable table;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry tv;
    private NetworkTableEntry ta;
    private NetworkTableEntry camMode;
    private NetworkTableEntry ledMode;
  

    public static LimelightSubsystem getInstance()
    {
      if (instance == null)
        instance = new LimelightSubsystem();
      
      return instance;
    }
  
    /**
     * Get limelight data from network table.
     */
    public LimelightSubsystem()
    {
      table = NetworkTableInstance.getDefault().getTable("limelight");
      tx = table.getEntry("tx"); 
      ty = table.getEntry("ty"); 
      tv = table.getEntry("tv"); 
      ta = table.getEntry("ta");

      ledMode = table.getEntry("ledMode"); // limelight's LED state (0-3).
      camMode = table.getEntry("camMode"); // limelight's operation mode (0-1).
    }
    
    @Override
    public void periodic() {
        //read values periodically
//        double x = getTargetOffsetX();
//        double y = getTargetOffsetY();
//        double area = getTargetArea();
//        boolean targetInView = targetInView();
        SmartDashboard.putNumber("AprilTag Distance", getDistanceToTarget());
    }

    public double getDistanceToTarget() {
        if(!targetInView()) return 0; // will return 0 if not in view

        // Find robot position to tag
        double[] botposeTargetSpace = getBotPoseTargetSpace();
        Pose3d botPositionFromTag = new Pose3d(botposeTargetSpace[0], botposeTargetSpace[1], botposeTargetSpace[2], new Rotation3d(botposeTargetSpace[3], botposeTargetSpace[4], botposeTargetSpace[5]));
        return Math.sqrt( Math.pow(botPositionFromTag.getX(), 2) + Math.pow(botPositionFromTag.getY(), 2) + Math.pow(botPositionFromTag.getZ(), 2) );
    }

    public double getTargetOffsetX()
    {
      return tx.getDouble(0.0);
    }
  
    public double getTargetOffsetY()
    {
      return ty.getDouble(0.0);
    }
  
    public double[] getBotPoseTargetSpace()
    {
      return NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_targetspace").getDoubleArray(new double[6]);
    }

    public boolean targetInView()
    {
        if (tv.getNumber(0).intValue() == 1) {
            return true;
        }
        return false;
    }
  
    public double getTargetArea()
    {
      return ta.getDouble(0.0);
    }
  
    private void setLEDMode(int mode)
    {
      ledMode.setNumber(mode);
    }
  

    public void turnOnLED()
    {
      this.setLEDMode(LimelightConstants.kledModeOn);
    }                 
    public void turnOffLED()
    {
      this.setLEDMode(LimelightConstants.kledModeOff);
    }
    public void blinkLED()
    {
      this.setLEDMode(LimelightConstants.kledModeBlink);
    }

  
    private void setCamMode(int mode)
    {
      camMode.setNumber(mode);
    }
  
    public void setModeDriver()
    {
      this.setLEDMode(LimelightConstants.kledModeOff);
      this.setCamMode(LimelightConstants.kcamModeDriverCamera);

    }
  
    public void setModeVision()
    {
      this.setLEDMode(LimelightConstants.kledModeOn);
      this.setCamMode(LimelightConstants.kcamModeVisionProcessor);
    }
  

    private boolean isDriverMode()
    {
      return ledMode.getDouble(0.0) == LimelightConstants.kledModeOff && camMode.getDouble(0.0) == LimelightConstants.kcamModeDriverCamera;
    }
    private boolean isVisionMode()
    {
      return ledMode.getDouble(0.0) == LimelightConstants.kledModeOn && camMode.getDouble(0.0) == LimelightConstants.kcamModeVisionProcessor;
    }
    
    public void toggleMode()
    {
      if (this.isDriverMode())
      {
        this.setModeVision();
      }
      else if (this.isVisionMode())
      {
        this.setModeDriver();
      }
      else
      {
        this.blinkLED();
      }
    }}
