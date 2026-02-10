package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

public class LimelightAutoAlignCommand extends Command {

    private final SwerveSubsystem swerve;
    private final String limelightName = "limelight";

    private final PIDController strafePID = new PIDController(0.03, 0.0, 0.002);
    private final PIDController rotPID = new PIDController(0.04, 0.0, 0.003);

    public LimelightAutoAlignCommand(SwerveSubsystem swerve) {
        this.swerve = swerve;

        strafePID.setTolerance(0.5);
        rotPID.setTolerance(1.0);

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        if (!LimelightHelpers.getTV(limelightName)) {
            swerve.drive(new Translation2d(), 0.0, true);
            return;
        }

        double tx = LimelightHelpers.getTX(limelightName);

        double strafeSpeed = -strafePID.calculate(tx, 0);
        double rotationSpeed = -rotPID.calculate(tx, 0);

        swerve.drive(
            new Translation2d(0.0, strafeSpeed),
            rotationSpeed,
            true
        );
    }

    @Override
    public boolean isFinished() {
        return strafePID.atSetpoint() && rotPID.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(), 0.0, true);
    }
}