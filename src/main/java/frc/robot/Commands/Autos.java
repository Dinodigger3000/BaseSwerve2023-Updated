package frc.robot.Commands;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.settings.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public final class Autos {
    private DrivetrainSubsystem drivetrain;
    public static SwerveAutoBuilder autoBuilder;
    private HashMap<String, Command> eventMap;

    private static Autos autos;

    private void Autos() {
    }

    public static Autos getInstance() {
        if (autos == null) {
            autos = new Autos();
        }
        return autos;
    }

    public void autoInit(SendableChooser<Command> autoChooser, HashMap<String, Command> eventMap, DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        this.eventMap = eventMap;
        // Create the AutoBuilder. This only needs to be created once when robot code
        // starts, not every time you want to create an auto command. A good place to
        // put this is in RobotContainer along with your subsystems.
        Autos.autoBuilder = new SwerveAutoBuilder(
                drivetrain::getPose, // Pose2d supplier
                drivetrain::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
                drivetrain.kinematics, // SwerveDriveKinematics
                new PIDConstants(
                        DriveConstants.k_XY_P,
                        DriveConstants.k_XY_I,
                        DriveConstants.k_XY_D), // PID constants to correct for translation error (used to create the X
                                                // and Y PID controllers)
                new PIDConstants(
                        DriveConstants.k_THETA_P,
                        DriveConstants.k_THETA_I,
                        DriveConstants.k_THETA_D), // PID constants to correct for rotation error (used to create the
                                                   // rotation controller)
                drivetrain::setModuleStates, // Module states consumer used to output to the drive subsystem
                eventMap,
                true, // Should the path be automatically mirrored depending on alliance color.
                       // Optional, defaults to true
                drivetrain // The drive subsystem. Used to properly set the requirements of path following
                           // commands
        );
        // add autos to smart dashboard.\
        autoChooser.addOption("example auto", ExampleAuto());
   
    }

    public SequentialCommandGroup ExampleAuto() {
        return new SequentialCommandGroup(
            autoBuilder.fullAuto(driveForward)
            );
    }

    // load all paths.
    static List<PathPlannerTrajectory> driveForward = PathPlanner.loadPathGroup("driveForward", new PathConstraints(2.5, 1.75));
}
