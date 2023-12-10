package frc.robot.Commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.settings.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public final class Autos {
    private DrivetrainSubsystem drivetrain;
    public static AutoBuilder autoBuilder;

    private static Autos autos;

    private void Autos() {
    }

    public static Autos getInstance() {
        if (autos == null) {
            autos = new Autos();
        }
        return autos;
    }

    public void autoInit(SendableChooser<Command> autoChooser, DrivetrainSubsystem drivetrain) {
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
                    0, //max module speed
                    1, //drive base radius
                    new ReplanningConfig()
                ),
                // drivetrain::setModuleStates, // Module states consumer used to output to the drive subsystem
                drivetrain // The drive subsystem. Used to properly set the requirements of path following
                           // commands
        );
        // add autos to smart dashboard.\
        autoChooser.addOption("example auto", ExampleAuto());
   
    }

    public SequentialCommandGroup ExampleAuto() {
        return new SequentialCommandGroup(
            new PathPlannerAuto("Example Auto")
            );
    }

  
    
}
