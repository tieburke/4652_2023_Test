package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class SetArmMidGoalCone extends CommandBase {

private final Arm arm;

  /** Creates a new SetShooterVelocity. */
  public SetArmMidGoalCone(Arm arm) {
    this.arm = arm;
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setWristToAngle(-7);
    if (arm.getWristPosition() < -10){
      arm.setElbowToAngle(-35);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (arm.getElbowPosition() < -85){
      return true;
    }
    return false;
  }
}