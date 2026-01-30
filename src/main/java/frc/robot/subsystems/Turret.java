package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.AngleUnit;


import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;

import edu.wpi.first.math.util.Units;
//import edu.wpi.first.units.Units;


import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightResults;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import limelight.networktables.target.pipeline.NeuralClassifier;

import static edu.wpi.first.units.Units.*;


public class Turret extends SubsystemBase
{

  Limelight limelight;
  LimelightPoseEstimator   limelightPoseEstimator;


  private final SparkMax                   turretMotor      = new SparkMax(Constants.turretMotor, MotorType.kBrushless);//, MotorType.kBrushless);
  private final SmartMotorControllerConfig motorConfig      = new SmartMotorControllerConfig(this)
      .withClosedLoopController(Constants.TurretkP, Constants.TurretkI, Constants.TurretkD, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
      .withSoftLimit(Degrees.of(-30), Degrees.of(100))
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      .withIdleMode(MotorMode.BRAKE)
      .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
      .withStatorCurrentLimit(Amps.of(80))
      .withMotorInverted(false)
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      .withFeedforward(new ArmFeedforward(0, 0, 0, 0)) //tune later using SysID. Note the current has gravity as a constant
      .withControlMode(ControlMode.CLOSED_LOOP);
      
  // private final SmartMotorController       motor            = new TalonFXSWrapper(turretMotor,
  //                                                                                 DCMotor.getNEO(1),
  //                                                                                 motorConfig);
   private final SmartMotorController       motor            = new SparkWrapper(turretMotor, DCMotor.getNeoVortex(1), motorConfig);

  private final MechanismPositionConfig  robotToMechanism = new MechanismPositionConfig()
      .withMaxRobotHeight(Meters.of(1.5))
      .withMaxRobotLength(Meters.of(0.75))
      .withRelativePosition(new Translation3d(Meters.of(-0.25), Meters.of(0), Meters.of(0.5)));
  private final PivotConfig                m_config         = new PivotConfig(motor)
      .withHardLimit(Degrees.of(-100), Degrees.of(200))
      .withTelemetry("TurretExample", TelemetryVerbosity.HIGH)
      .withStartingPosition(Degrees.of(0))
      .withMechanismPositionConfig(robotToMechanism);

  private final Pivot                      turret           = new Pivot(m_config);
  SwerveSubsystem swerve;


  //Setup the limelights
   public void setupLimelight()
  {

    limelight = new Limelight("limelight");
    limelight.getSettings()
             .withPipelineIndex(0)
             .withCameraOffset(new Pose3d(Units.inchesToMeters(0),
                                          Units.inchesToMeters(0),
                                          Units.inchesToMeters(0),
                                          new Rotation3d(0, 0, Units.degreesToRadians(0))))
             .withAprilTagIdFilter(List.of(17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11))
             .save();
  }

  public Turret(SwerveSubsystem swerve)
  {
    setupLimelight();
    this.swerve = swerve;
  }

  private int     outofAreaReading = 0;
  private boolean initialReading   = false;

  public void periodic()
  {
    turret.updateTelemetry();

    limelight.getSettings()
             .withRobotOrientation(new Orientation3d(new Rotation3d(swerve.getSwerveDrive().getOdometryHeading()
                                                                               .rotateBy(Rotation2d.kZero)),
                                                     new AngularVelocity3d(DegreesPerSecond.of(0),
                                                                           DegreesPerSecond.of(0),
                                                                           DegreesPerSecond.of(0))))
              .withCameraOffset(Constants.cameraOffsetFromRobotCenter.rotateAround(Constants.turretPivotCenterFromCamera, new Rotation3d(0, Degrees.of(65).in(Radians), turret.getAngle().in(Radians))))
             .save(); //camera pose is the camera pose from the center of robot
    Optional<PoseEstimate>     poseEstimates = limelightPoseEstimator.getPoseEstimate();
    Optional<LimelightResults> results       = limelight.getLatestResults();
    if (results.isPresent()/* && poseEstimates.isPresent()*/)
    {
        LimelightResults result       = results.get();
        PoseEstimate     poseEstimate = poseEstimates.get();
        SmartDashboard.putNumber("Avg Tag Ambiguity", poseEstimate.getAvgTagAmbiguity());
        SmartDashboard.putNumber("Min Tag Ambiguity", poseEstimate.getMinTagAmbiguity());
        SmartDashboard.putNumber("Max Tag Ambiguity", poseEstimate.getMaxTagAmbiguity());
        SmartDashboard.putNumber("Avg Distance", poseEstimate.avgTagDist);
        SmartDashboard.putNumber("Avg Tag Area", poseEstimate.avgTagArea);
        SmartDashboard.putNumber("Limelight Pose/x", poseEstimate.pose.getX());
        SmartDashboard.putNumber("Limelight Pose/y", poseEstimate.pose.getY());
        SmartDashboard.putNumber("Limelight Pose/degrees", poseEstimate.pose.toPose2d().getRotation().getDegrees());
        if (result.valid)
        {
          // Pose2d estimatorPose = poseEstimate.pose.toPose2d();
          Pose2d usefulPose     = result.getBotPose2d(Alliance.Blue);
          double distanceToPose = usefulPose.getTranslation().getDistance(swerve.getSwerveDrive().getPose().getTranslation());
          if (distanceToPose < 0.5 || (outofAreaReading > 10) || (outofAreaReading > 10 && !initialReading))
          {
            if (!initialReading)
            {
              initialReading = true;
            }
            outofAreaReading = 0;
            
            // System.out.println(usefulPose.toString());
            swerve.getSwerveDrive().setVisionMeasurementStdDevs(VecBuilder.fill(0.05, 0.05, 0.022));
            // System.out.println(result.timestamp_LIMELIGHT_publish);
            // System.out.println(result.timestamp_RIOFPGA_capture);
            swerve.getSwerveDrive().addVisionMeasurement(usefulPose, result.timestamp_RIOFPGA_capture);
          } else
          {
            outofAreaReading += 1;
          }
       }
    }
    }
  

  public void simulationPeriodic()
  {
    turret.simIterate();
  }

  public Command setPower(double dutycycle)
  {
    return turret.set(dutycycle);
  }

  public Command sysId()
  {
    return turret.sysId(Volts.of(3), Volts.of(3).per(Second), Second.of(30));
  }

  public Command setAngle(double angle)
  {
    return turret.setAngle(Angle.ofBaseUnits(angle, edu.wpi.first.units.Units.Degrees).mutableCopy());
  }

  public double getAngle()
  {
    return turret.getAngle().magnitude();
  }
}