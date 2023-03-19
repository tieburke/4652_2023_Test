package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TankDrive extends CommandBase {
  Drivetrain drivetrain;
  DoubleSupplier forward, rotation, throttle;
  BooleanSupplier invert;

  public TankDrive(DoubleSupplier forward, DoubleSupplier rotation, DoubleSupplier throttle, Drivetrain drivetrain){
    
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
   
    this.forward = forward;
    this.rotation = rotation;
    this.throttle = throttle;
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    double speed = forward.getAsDouble();
    double rotate = rotation.getAsDouble();
    double speedControl = throttle.getAsDouble();
    
    drivetrain.drive(speed, rotate);
    //drivetrain.drive(speed*speedControl, rotate*speedControl);
 
    SmartDashboard.putNumber("rotate", rotate);
    SmartDashboard.putNumber("forward", speed);
    SmartDashboard.putNumber("speedControl", speedControl);

    if (Math.abs(speed + rotate) < 0.1){
      drivetrain.setDriveMotors(0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //drivetrain.drive(0, 0);
    drivetrain.setDriveMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
