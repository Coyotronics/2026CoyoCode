package frc.robot.commands;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TurretVision;
import frc.robot.subsystems.Turret;
import edu.wpi.first.math.controller.PIDController;

public class AutoAlignTurret extends Command {

    private Turret turret;
    private TurretVision vision;
    private PIDController pid;

    private int kP;
    private int kI;
    private int kD;

    public AutoAlignTurret(Turret turret, TurretVision vision) {
        this.turret = turret;
        this.vision = vision;
        this.pid = new PIDController(kP, kI, kD);
        addRequirements(turret);
    }

    @Override
    public void execute() {
        if (vision.hasTarget()) {
            double correction = pid.calculate(vision.getTargetX(), 0.0); // Not sure if PIDs are necessary here

            double currentAngle = turret.getAngle();
            double desiredAngle = currentAngle + correction;

            //double safeTarget = getSafeTarget(currentAngle, desiredAngle);

            //turret.setAbsoluteSetpoint(safeTarget);
            turret.setAngle(desiredAngle);

            //turret.setClosedLoopPosition();
            
        } 
        else 
        {
            turret.setAngle(turret.getAngle());
            //turret.setClosedLoopPosition();
        }
    }


    public double getSafeTarget(double currentAngle, double desiredAngle) 
    {
        double delta = desiredAngle - currentAngle;
        double safeTarget = desiredAngle;
        
        if(desiredAngle > 180)
            safeTarget = (desiredAngle % 180) - 180;
        if(desiredAngle < -180)
            safeTarget = -((-desiredAngle % 180) - 180);

        return safeTarget;
    }
}