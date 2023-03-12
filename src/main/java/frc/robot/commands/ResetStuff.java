package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ResetStuff extends CommandBase {

    private final Drivetrain drivetrain;

    public ResetStuff(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

    }

    @Override
    public void initialize() {
    
        drivetrain.forgetAngle();
        drivetrain.resetEncoders();
        drivetrain.resetOdometry(new Pose2d());

    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {

        return true;

    }
}