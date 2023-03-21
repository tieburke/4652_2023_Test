package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AutoDrive extends CommandBase {

    private final Drivetrain drivetrain;

    private double power, distance, angle, startEncoder;

    public AutoDrive(double power, double distance, double angle, Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        this.power = power;
        this.distance = distance;
        this.angle = angle;

    }

    @Override
    public void initialize() {
        startEncoder = drivetrain.getEncoderAverage();
        drivetrain.forgetAngle();
        angle = (Math.abs(angle) - 12)*angle/Math.abs(angle);
        //drivetrain.m_leftGroup.setInverted(true);
    }

    @Override
    public void execute() {
        //if(angle == 0){
            drivetrain.drive(power, 0);
        //}
        //if(angle != 0){
        //    drivetrain.drive(0, angle);
        //}

        SmartDashboard.putBoolean("Isfinished?", isFinished());
        SmartDashboard.putNumber("startEncoder + distance", startEncoder + distance);
        SmartDashboard.putNumber("getEncoderAverage", drivetrain.getEncoderAverage());
        SmartDashboard.putNumber("RobotAngle", drivetrain.getYaw().getDegrees());
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setDriveMotors(0);
        SmartDashboard.putBoolean("interrupted", interrupted);
        //drivetrain.m_rightGroup.setInverted(true);
    }

    @Override
    public boolean isFinished() {
        //if (angle == 0){
            if(drivetrain.getEncoderAverage() >= startEncoder + distance && power > 0) {
                return true;
            }
            if(drivetrain.getEncoderAverage() <= startEncoder - distance && power < 0) {
                return true;
            }
        //}

        // if (angle != 0 && Math.abs(drivetrain.getYaw().getDegrees()) - Math.abs(angle) >= 0){
            // return true;
        // }
        
        return false;
    }
}