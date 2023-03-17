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
import frc.robot.commands.AutoDrive;
import frc.robot.commands.Balance;
import frc.robot.commands.ManualElbowDriveDown;
import frc.robot.commands.ManualElbowDriveUp;
import frc.robot.commands.ManualWristDriveClockwise;
import frc.robot.commands.ManualWristDriveCounterclockwise;
import frc.robot.commands.ResetStuff;
import frc.robot.commands.SetArmInitialize;
import frc.robot.commands.SetArmLowGoal;
import frc.robot.commands.SetArmMidGoalCone;
import frc.robot.commands.SetArmMidGoalCube;
import frc.robot.commands.SetArmPickup;
import frc.robot.commands.TankDrive;
import frc.robot.commands.TestPath;
import frc.robot.subsystems.Arm;


public class RobotContainer {
    
    private final Joystick stick;
    private final XboxController xbox;
    private final Drivetrain drivetrain;
    private final Arm arm;
    private SendableChooser<Command> chooser;

    final JoystickButton j1, j2, j3, j4, j5, j6, j7, j8, j9, j10, j11, j12;
    final JoystickButton xA, xB, xX, xY, xRB, xLB, xSelect, xMenu, x9, x10;

    private int wristAxis = XboxController.Axis.kLeftY.value;
    private int elbowAxis = XboxController.Axis.kRightY.value;

    private double initAngle;

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
    
        configureButtonBindings();
    
        initializeAutoChooser();

        initAngle = drivetrain.getRoll();

    }
        
        private void setDefaultCommands(){
            drivetrain.setDefaultCommand(new TankDrive(() -> (filter(stick.getY()))*2, () -> (filter(-stick.getTwist()))*3, drivetrain));//xbox.getRawAxis(elbowAxis), xbox.getRawAxis(elbowAxis), drivetrain, arm));}
        }

        private void configureButtonBindings() {
            j1.whileTrue(new InstantCommand(arm::clawToggle, arm));
            j2.onTrue(new InstantCommand(drivetrain::switchGears, drivetrain));
            j3.whileTrue(new ManualWristDriveCounterclockwise(arm));
            j4.onTrue(new InstantCommand(() -> drivetrain.forgetAngle()));
            j5.onTrue(new InstantCommand(() -> drivetrain.resetEncoders()));
            j6.onTrue(new InstantCommand(arm::resetElbowAngle, arm));
            j7.onTrue(new InstantCommand(arm::resetWristAngle, arm));
            j8.onTrue(new InstantCommand(() -> drivetrain.resetOdometry(new Pose2d())));
            j9.onTrue(new InstantCommand(drivetrain::disableCompressor, drivetrain));
            j10.onTrue(new InstantCommand(drivetrain::enableCompressor, drivetrain));
            
            xX.onTrue(new SetArmMidGoalCone(arm));
            xY.onTrue(new SetArmLowGoal(arm));
            xB.onTrue(new SetArmInitialize(arm));
            xA.onTrue(new SetArmPickup(arm));
            //xA.whileTrue(new ManualElbowDriveUp(arm));
            //xB.whileTrue(new ManualElbowDriveDown(arm));
            //xX.whileTrue(new ManualWristDriveCounterclockwise(arm));
            //xY.whileTrue(new ManualWristDriveClockwise(arm));
            xLB.onTrue(new InstantCommand(arm::clawToggle, arm));
            xRB.onTrue(new InstantCommand(arm::pressureToggle, arm));
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

            chooser.addOption("placeThenDriveOutArea",
            new SequentialCommandGroup(
                new SetArmMidGoalCube(arm),
                new WaitCommand(2),
                new InstantCommand(arm::clawToggle, arm),
                new WaitCommand(2),
                new SetArmMidGoalCone(arm),
                new WaitCommand(0.5),
                new ParallelCommandGroup(
                    new SetArmInitialize(arm),
                    new AutoDrive(1, 150, 0, drivetrain)
                )
            )
            );

            chooser.addOption("placeThenBalance",
            new SequentialCommandGroup(
                // new SetArmMidGoalCube(arm),
                // new WaitCommand(2),
                // new InstantCommand(arm::clawToggle, arm),
                // new WaitCommand(2),
                // new SetArmMidGoalCone(arm),
                // new WaitCommand(0.5),
                // new ParallelCommandGroup(
                    // new SetArmInitialize(arm),
                    // new AutoDrive(1, 150, 0, drivetrain)
                    // ),
                // new AutoDrive(-1, -50, 0, drivetrain),
                new Balance(drivetrain, initAngle)
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
            //tiernan is a munch
        public Command getAutonomousCommand() {
            return chooser.getSelected();
        }

}