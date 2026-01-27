/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//credit to https://github.com/4201VitruvianBots/TRex2021/blob/main/src/main/java/frc/robot/subsystems/Turret.java

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
// import com.revrobotics.CANEncoder;
// import com.revrobotics.CANPIDController;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.CANSparkMax.SoftLimitDirection;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.geometry.Pose2d;
// import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
// import frc.robot.simulation.SimConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;



/*
Subsystem for controlling the turret
 */

public class Turret extends SubsystemBase {
    private final SwerveSubsystem m_swerveDrive;

    // setup motor and encoder variables
    SparkMax turretMotor = new SparkMax(Constants.turretMotor, MotorType.kBrushless);
    RelativeEncoder encoder = turretMotor.getEncoder();
    SparkClosedLoopController pidController = turretMotor.getClosedLoopController();


    // Turret PID gains
    double kF = 0.000006;
    double kP = 0.00002;
    double kI = 0.000;
    double kD = 0.0;
    int kI_Zone = 4;
    int kErrorBand = 1;//degreesToEncoderUnits(0.5);
    int kCruiseVelocity = 14000;
    int kMotionAcceleration = kCruiseVelocity * 10;

    // setup variables
    double minAngle = -20;
    double maxAngle = 20;
    private final int encoderUnitsPerRotation = 42;
    double gearRatio = 27.0 / 120.0;    // TODO: Ratio is correct, but values are wrong
    private double setpoint = 0; //angle
    private int controlMode = 1;
    private boolean initialHome;
    private boolean turretHomeSensorLatch = false;

    // AutoAlign Variables
    private double lastAngle = 0.0; // The angle from the previous looop
    private double turretAbsoluteAngle = 0.0; // Absolute angle turn of the turret to determine turn direction
    private static final double MAX_TWIST_DEG = 360.0;

    public Turret(SwerveSubsystem swerveDrive) {
        this.m_swerveDrive = swerveDrive;

        SparkMaxConfig config = new SparkMaxConfig();

        // Basic motor settings
        config.inverted(true);            // same as motor.setInverted(true)
        config.idleMode(IdleMode.kBrake); // brake mode
        config.smartCurrentLimit(80);     // current limiting (80 bc vortexes)



        // Now configure closed‑loop at slot 0
        ClosedLoopConfig cl = config.closedLoop;
        cl.pid( Constants.TurretkP, Constants.TurretkI, Constants.TurretkD ); 
        
        MAXMotionConfig mmConfig = new MAXMotionConfig();
        mmConfig.cruiseVelocity(Constants.TurretCruiseVelocity);
        
      
        // Apply config to motor
        turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Grab the closed‑loop controller
        pidController = turretMotor.getClosedLoopController();


        //encoder shi
        resetEncoder();
        encoder.setPosition(0);

        
        initElastic();

        // Setup turret motors
        // turretMotor.restoreFactoryDefaults();
        // turretMotor.setInverted(true);
        // turretMotor.setIdleMode(IdleMode.kBrake);
        // turretMotor.setSmartCurrentLimit(20, 20);
        // // turretMotor.setSoftLimit(SoftLimitDirection.kForward, (float)0.25);
        // // turretMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)-0.25);

        // // Setup PID Controller
        // pidController.setFF(kF);
        // pidController.setP(kP);
        // pidController.setI(kI);
        // pidController.setD(kD);
        // double maxVel = 1.1e4;
        // pidController.setSmartMotionMaxVelocity(maxVel, 0); // Formerly 1.1e4
        // double maxAccel = 1e6;
        // pidController.setSmartMotionMaxAccel(maxAccel, 0); // Formerly 1e6
        // pidController.setSmartMotionAllowedClosedLoopError(kErrorBand, 0);
        // pidController.setIZone(kI_Zone);
    }

    // self-explanatory commands
    public void resetEncoder() {
        encoder.setPosition(0);
    }

    public int getControlMode() {
        return controlMode;
    }

    public void setControlMode(int mode) {
        controlMode = mode;
    }

    public double getTurretAngle() {
        return encoderUnitsToDegrees(encoder.getPosition());
    }

    public double getRobotRelativeAngle() {
        return getTurretAngle() - m_swerveDrive.getHeading().getDegrees();
    }

    public double getMaxAngle() {
        return maxAngle;
    }

    public double getMinAngle() {
        return minAngle;
    }

    public boolean getTurretHome() {
//        return !turretHomeSensor.get(); cannot understand what the turret home sensor is refering to.
        return getTurretAngle() == 0;
    }

    public boolean getInitialHome() { //Checks if the bot is in its starting position??
        return initialHome;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public double getError() {
        return getSetpoint() - getTurretAngle();
    }

    public void setPercentOutput(double output) {
        turretMotor.set(output);
    }

    public void setAbsoluteSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void setRobotCentricSetpoint(double setpoint) {
        setpoint -= m_swerveDrive.getHeading().getDegrees(); // + m_swerveDrive.getHeadingOffset(); //heading offset appears to have been a constant. Maybe their heading was always off by 180 degrees?

        if (setpoint > getMaxAngle())
            setpoint -= 360;
        else if (setpoint < getMinAngle())
            setpoint += 360;

        this.setpoint = setpoint;
    }

    public void setClosedLoopPosition() {
        double angle = Math.max(Math.min(getSetpoint(), maxAngle),  minAngle);

        pidController.setSetpoint(degreesToEncoderUnits(angle), ControlType.kMAXMotionPositionControl);  //formerly ControlType.kSmartMotion
    }

    public int degreesToEncoderUnits(double degrees) {
        return (int) (degrees * (1.0 / gearRatio) * (encoderUnitsPerRotation / 360.0));
    }

    public double encoderUnitsToDegrees(double encoderUnits) {
        return encoderUnits * gearRatio * (360.0 / encoderUnitsPerRotation);
    }

    // checks if the turret is pointing within the tolerance of the target
    public boolean onTarget() {
       return pidController.isAtSetpoint();
    }

    private boolean getTurretLatch() {
        return turretHomeSensorLatch;
    }

    private void setTurretLatch(boolean state) {
        turretHomeSensorLatch = state;
    }


    NetworkTable table;
    NetworkTableEntry setpointEntry;
    NetworkTableEntry angleEntry ;
    NetworkTableEntry errorEntry;   
    NetworkTableEntry homeEntry;

    private void initElastic() {
        table = NetworkTableInstance.getDefault().getTable("Turret");

        setpointEntry = table.getEntry("Turret Setpoint");
        angleEntry    = table.getEntry("Turret Angle");
        errorEntry    = table.getEntry("Turret Error");
        homeEntry     = table.getEntry("Turret Home");
    }

    // set elastic
    private void updateElastic() {

        setpointEntry.setDouble(getSetpoint());
        angleEntry.setDouble(getTurretAngle());
        errorEntry.setDouble(getError());
        homeEntry.setBoolean(getTurretHome());
    }

    @Override
    public void periodic() {

        if (getControlMode() == 1)
            setClosedLoopPosition();

        // This method will be called once per scheduler run
        // TODO: FIX
        if (!getTurretLatch() && getTurretHome()) {
//            turretMotor.setSelectedSensorPosition(0); no sensor on the CANSparkmax
            encoder.setPosition(0);
            setTurretLatch(true);
        } else if (getTurretLatch() && !getTurretHome())
            setTurretLatch(false);

        if (!getInitialHome())
            if (getTurretHome())
                initialHome = true;

        updateElastic();
    }

    public double getTurretSimAngle(){
        return getTurretAngle() + 180;
    }

    public Pose2d getTurretSimPose() {
        return new Pose2d(m_swerveDrive.getPose().getX(),
                m_swerveDrive.getPose().getY(),
                new Rotation2d(Math.toRadians(getTurretSimAngle())));
    }

    @Override
    public void simulationPeriodic() {
    }

    public double getTurretAbsoluteAngle(){
        return turretAbsoluteAngle;
    }


    //Simulation stuff. will have to get running eventually
    
    // public double getIdealTargetDistance() {
    //     return Math.sqrt(Math.pow(SimConstants.blueGoalPose.getY() - getTurretSimPose().getY(), 2) + Math.pow(SimConstants.blueGoalPose.getX() - getTurretSimPose().getX(), 2));
    // }

    // public double getIdealTurretAngle() {

    //     double targetRadians = Math.atan2(SimConstants.blueGoalPose.getY() - getTurretSimPose().getY(), SimConstants.blueGoalPose.getX() - getTurretSimPose().getX());

    //     return Math.toDegrees(targetRadians);
    // }
}