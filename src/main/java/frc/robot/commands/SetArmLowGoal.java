package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class SetArmLowGoal extends CommandBase {

private final Arm arm;

  /** Creates a new SetShooterVelocity. */
  public SetArmLowGoal(Arm arm) {
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
    arm.setElbowToAngle(-40);
    //arm.setElbowToAngle(180);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}