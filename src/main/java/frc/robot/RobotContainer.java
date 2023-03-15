package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.ManualElbowDriveDown;
import frc.robot.commands.ManualElbowDriveUp;
import frc.robot.commands.ManualWristDriveClockwise;
import frc.robot.commands.ManualWristDriveCounterclockwise;
import frc.robot.commands.ResetStuff;
import frc.robot.commands.SetArmLowGoal;
import frc.robot.commands.TankDrive;
import frc.robot.commands.autocommands.AutoDrive;
import frc.robot.commands.autocommands.AutoPaths.TestPath;
import frc.robot.subsystems.Arm;


public class RobotContainer {
    
    private final Joystick stick;
    private final XboxController xbox;
    private final Drivetrain drivetrain;
    private final Arm arm;
    private SendableChooser<Command> chooser;

    final JoystickButton j1, j2, j3, j4, j5, j6, j7, j8, j9, j10, j11, j12;
    final JoystickButton xA, xB, xX, xY, xRB, xLB, xSelect, xMenu, x9, x10;

    public RobotContainer() {
        stick = new Joystick(0);
        xbox = new XboxController(1);
        drivetrain = new Drivetrain();
        arm = new Arm();
        chooser = new SendableChooser<>();

        j1 = new JoystickButton(stick, 1);
        j2 = new JoystickButton(stick, 2);
        j3 = new JoystickButton(stick, 3);
        j4 = new JoystickButton(stick, 4);
        j5 = new JoystickButton(stick, 5);
        j6 = new JoystickButton(stick, 6);
        j7 = new JoystickButton(stick, 7);
        j8 = new JoystickButton(stick, 8);
        j9 = new JoystickButton(stick, 9);
        j10 = new JoystickButton(stick, 10);
        j11 = new JoystickButton(stick, 11);
        j12 = new JoystickButton(stick, 12);
    
        xA = new JoystickButton(xbox, 1);
        xB = new JoystickButton(xbox, 2);
        xX = new JoystickButton(xbox, 3);
        xY = new JoystickButton(xbox, 4);
        xLB = new JoystickButton(xbox, 5);
        xRB = new JoystickButton(xbox, 6);
        xMenu = new JoystickButton(xbox, 7);
    
        xSelect = new JoystickButton(xbox, 8);
        x9 = new JoystickButton(xbox, 9);
        x10 = new JoystickButton(xbox, 10);
        
        setDefaultCommands();
    
        // Configure the button bindings
        configureButtonBindings();
    
        initializeAutoChooser();

    }
        
        private void setDefaultCommands(){
            drivetrain.setDefaultCommand(new TankDrive(() -> filter(stick.getY())*3, () -> filter(stick.getTwist())*-4, drivetrain));
        }

        private void configureButtonBindings() {
            j1.onTrue(new SetArmLowGoal(arm));
            j2.onTrue(new InstantCommand(drivetrain::switchGears, drivetrain));
            j3.onTrue(new InstantCommand(() -> drivetrain.resetOdometry(new Pose2d())));
            j4.onTrue(new InstantCommand(() -> drivetrain.forgetAngle()));
            j5.onTrue(new InstantCommand(() -> drivetrain.resetEncoders()));
            xA.whileTrue(new ManualElbowDriveUp(arm));
            xB.whileTrue(new ManualElbowDriveDown(arm));
            xX.whileTrue(new ManualWristDriveCounterclockwise(arm));
            xY.whileTrue(new ManualWristDriveClockwise(arm));
        }

        public double filter(double value) {
            if(Math.abs(value) < 0.5) {
              value = 0;
            }
            return value; 
        }

        public void initializeAutoChooser() {
    
            chooser.setDefaultOption(
                    "nothing",
              new WaitCommand(10)
                );

            chooser.addOption("drive",
            new SequentialCommandGroup(
                new AutoDrive(-0.7, 50, 0, drivetrain)
            )
            );

            chooser.addOption("turn?",
            new SequentialCommandGroup(
                new AutoDrive(0.3, 0, -180, drivetrain)
            )
            );

            chooser.addOption("TestPath",
            new SequentialCommandGroup(
                new ResetStuff(drivetrain),
                getRamseteCommand(TestPath.getTraj(drivetrain)))
            );
            SmartDashboard.putData(chooser);
        }

        public RamseteCommand getRamseteCommand(Trajectory trajectory) {
            RamseteCommand ramseteCommand = new RamseteCommand(trajectory, drivetrain::getPose, drivetrain.getRamseteController(),
                                                                drivetrain.getFeedforward(), drivetrain.getKinematics(),
                                                                drivetrain::getWheelSpeeds, drivetrain.getLeftPIDController(),
                                                                drivetrain.getRightPIDController(),
                                                                drivetrain::tankDriveVolts, drivetrain);
            return ramseteCommand;
        }

        public Command getAutonomousCommand() {
            return chooser.getSelected();
        }

}