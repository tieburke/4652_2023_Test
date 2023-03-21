package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;



public class Balance extends CommandBase {
  /** Creates a new BalanceRobot Command */

  private Drivetrain drivetrain;
  private Timer timer;
  private double corrections = 1;
  private boolean direction = true, lastDirection;

  public Balance(Drivetrain drivetrain) {
    timer = new Timer();
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {

    SmartDashboard.putNumber("Timer", timer.get());

    lastDirection = direction;

    if (drivetrain.getRoll() > 0){
      drivetrain.drive(0.4*corrections, 0);
      direction = true;
      SmartDashboard.putNumber("corrections", corrections);
      SmartDashboard.putBoolean("direction", direction);
    }

    else if (drivetrain.getRoll() < 0){
      drivetrain.drive(-0.4*corrections, 0);
      direction = false;
      SmartDashboard.putNumber("corrections", corrections);
      SmartDashboard.putBoolean("direction", direction);
    }

    if (direction == false && lastDirection == true){
      corrections = corrections/2;
    }
    else if (direction == true && lastDirection == false){
      corrections = corrections/2;
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setDriveMotors(0);
    timer.stop();
    timer.reset();
    SmartDashboard.putBoolean("balancedFinished", true);
  }


  @Override
  public boolean isFinished() {
  
  if (Math.abs(drivetrain.getRoll()) < 1 && timer.get() > 6){
    return true;
  }
  
    return false;
  }
}