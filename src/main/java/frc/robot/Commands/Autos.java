package frc.robot.Commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import static frc.robot.settings.Constants.DriveConstants.*;


public final class Autos {
    private DrivetrainSubsystem drivetrain;
    public static AutoBuilder autoBuilder;
    public static SendableChooser<Command> autoChooser;
    private static Autos autos;

    private void Autos() {
    }

    public static Autos getInstance() {
        if (autos == null) {
            autos = new Autos();
        }
        return autos;
    }

    public void autoInit(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        // Create the AutoBuilder. This only needs to be created once when robot code
        // starts, not every time you want to create an auto command. A good place to
        // put this is in RobotContainer along with your subsystems.
        AutoBuilder.configureHolonomic(
                drivetrain::getPose, // Pose2d supplier
                drivetrain::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
                drivetrain::getChassisSpeeds,
                drivetrain::drive,
                new HolonomicPathFollowerConfig(
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
                    4, //max module speed //TODO find actual values
                    new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0).getNorm(), //drive base radius
                    new ReplanningConfig()
                ),
                // drivetrain::setModuleStates, // Module states consumer used to output to the drive subsystem
                drivetrain // The drive subsystem. Used to properly set the requirements of path following
                           // commands
        );
        autoChooser = AutoBuilder.buildAutoChooser(); // automatically add all autos to a new chooser!
        // add autos to smart dashboard.\
        SmartDashboard.putData("AutoChooser", autoChooser);
        SmartDashboard.putString("autos", AutoBuilder.getAllAutoNames().toString());
        
        // autoChooser.addOption("example auto", ExampleAuto());//not needed anymore, unless you want to add a NON-pathplanner auto
    }
    
    public Command getBeanPath() {
        PathPlannerPath bean = PathPlannerPath.fromPathFile("Beans");
        return AutoBuilder.pathfindThenFollowPath(bean, Constants.DriveConstants.DEFAUL_PATH_CONSTRAINTS, 0.25);
    }

    
    /**A relic of old code, instead of calling this in RobotContainer, use "new PathPlannerAuto("Example Auto")" without having to call this method. */
    // public SequentialCommandGroup ExampleAuto() {
    //     return new SequentialCommandGroup(
    //         new PathPlannerAuto("Example Auto")
    //         ); 
    // }

  
    
}
