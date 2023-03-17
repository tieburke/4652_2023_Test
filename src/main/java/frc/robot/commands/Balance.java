package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;



public class Balance extends CommandBase {
  /** Creates a new BalanceRobot Command */

  private Drivetrain drivetrain;
  private Timer timer;
  private double initAngle;

  public Balance(Drivetrain drivetrain, double initAngle) {
    timer = new Timer();
    this.initAngle = initAngle;
    this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(isBalanced() == -1){
      drivetrain.drive(-0.2, 0);
    } else if(isBalanced() == 1){
        drivetrain.drive(0.2, 0);
    } else {
      SmartDashboard.putNumber("BalanceTimer", timer.get());
      drivetrain.drive(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0);
    timer.stop();
    timer.reset();
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(isBalanced() == 0 && timer.hasElapsed(2)){
      return true;
    } else {
      return false;
    }


  }

  public int isBalanced(){
    if(drivetrain.getRoll() < (initAngle -0.5)){
        timer.reset();
        return -1;
    } else if(drivetrain.getRoll() > (initAngle + 0.5)){
        timer.reset();
        return 1;
    } else {
        return 0;
    }
}

}