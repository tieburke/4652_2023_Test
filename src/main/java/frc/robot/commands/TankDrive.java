package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Arm;


public class TankDrive extends CommandBase {
  Drivetrain drivetrain;
  //Arm arm;
  DoubleSupplier forward, rotation;// elbowSpeed, wristSpeed;
  BooleanSupplier invert;

  public TankDrive(DoubleSupplier forward, DoubleSupplier rotation, Drivetrain drivetrain){//DoubleSupplier elbowSpeed, DoubleSupplier wristSpeed, Drivetrain drivetrain, Arm arm) {
    
    this.drivetrain = drivetrain;
    //this.arm = arm;
    addRequirements(drivetrain);
    //addRequirements(arm);

    this.forward = forward;
    this.rotation = rotation;
    //this.elbowSpeed = elbowSpeed;
    //this.wristSpeed = wristSpeed;
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
    //double elbowPower = elbowSpeed.getAsDouble();
    //double wristPower = wristSpeed.getAsDouble();
    
    drivetrain.drive(speed, rotate);

    //arm.setElbowSpeed(elbowPower);
    //arm.setWristSpeed(wristPower);
    
    SmartDashboard.putNumber("rotate", rotate);
    SmartDashboard.putNumber("forward", speed);

    if (Math.abs(speed + rotate) < 0.1){
      drivetrain.drive(0,0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
