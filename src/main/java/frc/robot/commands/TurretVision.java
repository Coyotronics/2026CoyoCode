package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class TurretVision extends SubsystemBase {
    private final double MAX_ANGULAR_VELOCITY = 0.0;

    // In-place vision variables
    private double targetX; // Horizontal offset
    private double targetY; // Vertical offset
    private boolean hasTarget;
    private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    // AprilTag ID
    private final int RED_ALLIANCE_HUB_ID = 10;
    private final int BLUE_ALLIANCE_HUB_ID = 26; // Could be 25, cross-check later
    private double tagID;
    private double desiredTag;

    // What Alliance?
    private final boolean isRed = false;

    private SwerveSubsystem swerveSubsystem;

    public TurretVision(SwerveSubsystem swerveSubsystem) 
    {
        targetX = Double.POSITIVE_INFINITY;
        targetY = 0;
        hasTarget = false;

        this.swerveSubsystem = swerveSubsystem;

        initElastic();
    }

    NetworkTable table;
    NetworkTableEntry targetXEntry;
    NetworkTableEntry TargetYEntry ;
    NetworkTableEntry hasTargetEntry;

    private void initElastic(){
        table = NetworkTableInstance.getDefault().getTable("TurretVision");

        targetXEntry = table.getEntry("Turret Target X");
        TargetYEntry    = table.getEntry("Turret Target Y");
        hasTargetEntry    = table.getEntry("Turret Has Target");
    }

    private void updateElastic(){
        targetXEntry.setDouble(getTargetX());
        TargetYEntry.setDouble(getTargetY());
        hasTargetEntry.setBoolean(hasTarget());

    }
    @Override
    public void periodic() {
        updateVisionData();
        updateElastic();
        // SmartDashboard.putNumber("Turret Target X", targetX);
        // SmartDashboard.putNumber("Turret Target Y", targetY);
        // SmartDashboard.putBoolean("Turret Has Target", hasTarget);
    }

    private void updateVisionData() {
        

        hasTarget = limelightTable.getEntry("tv").getDouble(0.0) == 1.0;
        tagID = limelightTable.getEntry("tid").getDouble(-1.0);


        // Will have to change this to be based off of driverStation and not random boolean
        if(isRed)
            desiredTag = RED_ALLIANCE_HUB_ID;
        else
            desiredTag = BLUE_ALLIANCE_HUB_ID;


        if (hasTarget && tagID == desiredTag) 
        {
            // horizontal angle to target
            targetX = limelightTable.getEntry("tx").getDouble(0.0); 
            // vertical angle to target
            targetY = limelightTable.getEntry("ty").getDouble(0.0); 
        }
        else 
        {
            targetX = 0;
            targetY = 0;
        }
    }

    // getters
    public double getTargetX() {
        return targetX;
    }

    public double getTargetY() {
        return targetY;
    }

    public boolean hasTarget() {
        return hasTarget;
    }
}