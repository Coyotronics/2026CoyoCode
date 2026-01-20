package frc.robot.commands;

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

            double currentAngle = turret.getTurretAbsoluteAngle();
            double desiredAngle = currentAngle + correction;

            //double safeTarget = getSafeTarget(currentAngle, desiredAngle);

            //turret.setAbsoluteSetpoint(safeTarget);
            turret.setAbsoluteSetpoint(desiredAngle);

            //turret.setClosedLoopPosition();
            
        } 
        else 
        {
            turret.setAbsoluteSetpoint(turret.getTurretAbsoluteAngle());
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