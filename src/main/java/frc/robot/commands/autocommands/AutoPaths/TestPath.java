package frc.robot.commands.autocommands.AutoPaths;

import java.io.IOException;
import java.nio.file.Path;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.Drivetrain;

public final class TestPath {
    public static Trajectory getTraj(Drivetrain drivetrain) {


      String swervyJSON = "output/Swervy.wpilib.json";
      Trajectory swervyTraj = new Trajectory();

      try {
        Path swervyPath = Filesystem.getDeployDirectory().toPath().resolve(swervyJSON);
        swervyTraj = TrajectoryUtil.fromPathweaverJson(swervyPath);
     } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + swervyJSON, ex.getStackTrace());
     }

      drivetrain.resetOdometry(swervyTraj.getInitialPose());
      return swervyTraj;
    }
    
}