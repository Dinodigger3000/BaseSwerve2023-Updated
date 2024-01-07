// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.settings.LimelightDetectorData;
import frc.robot.settings.Constants.DriveConstants;
import frc.robot.settings.Constants.Vision;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;

public class DriveToCube extends CommandBase {
  private final DrivetrainSubsystem drivetrain;
  private final Limelight ll;

  private LimelightDetectorData detectorData;
  private double tx;
  private double ta;
  private PIDController txController;
  private PIDController taController;
  private SlewRateLimiter txLimiter;
  private SlewRateLimiter taLimiter;

  /** Creates a new DrivePickupCube. */
  public DriveToCube(DrivetrainSubsystem drivetrain) {
    this.drivetrain = drivetrain;
    this.ll = Limelight.getInstance();

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    txController = new PIDController(
        Vision.K_DETECTOR_TX_P,
        Vision.K_DETECTOR_TX_I,
        Vision.K_DETECTOR_TX_D);
    taController = new PIDController(
        Vision.K_DETECTOR_TA_P,
        Vision.K_DETECTOR_TA_I,
        Vision.K_DETECTOR_TA_D);
    taLimiter = new SlewRateLimiter(1, -1, 0);
    txController.setSetpoint(0);
    taController.setSetpoint(0);
    txController.setTolerance(1, 0.25);
    taController.setTolerance(1, 0.25);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    detectorData = Limelight.latestDetectorValues;
    if (detectorData == null) {
      drivetrain.stop();
      System.err.println("nullDetectorData");
      return;
    }
    if (!detectorData.isResultValid) {
      drivetrain.stop();
      System.err.println("invalidDetectorData");
      return;
    }
    
    tx = detectorData.tx;
    ta = detectorData.ta;

    SmartDashboard.putNumber("Ttx", txController.calculate(tx));
    SmartDashboard.putNumber("Tty", taController.calculate(ta));

    if (taController.atSetpoint() && txController.atSetpoint()) {
      drivetrain.stop();
    } else {
      drivetrain.drive(new ChassisSpeeds(taLimiter.calculate(taController.calculate(ta)), 0, txController.calculate(tx)));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (tx == 0 && ta == 0) System.out.println("There might be a problem with the limelight detector");
    if (interrupted) System.out.println("DriveToCube interrupted");
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (taController.atSetpoint() && txController.atSetpoint());
  }
}
