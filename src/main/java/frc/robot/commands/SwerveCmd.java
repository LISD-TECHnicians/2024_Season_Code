package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.SwerveSubsystem;

import frc.robot.Constants.DriveConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveCmd extends Command {
  private final SwerveSubsystem swerveSubsystem;

  private final DoubleSupplier xController; // m/s
  private final DoubleSupplier yController; // m/s
  private final DoubleSupplier rotationController; // rad/s

  private final BooleanSupplier robotOriented;

  private final PIDController rotationPositionPID = new PIDController(DriveConstants.ROTATION_POSITION_CONTROL_P, 
      DriveConstants.ROTATION_POSITION_CONTROL_I, DriveConstants.ROTATION_POSITION_CONTROL_D);

  private ChassisSpeeds swerveSpeeds = new ChassisSpeeds(); 

  public SwerveCmd(SwerveSubsystem swerveSubsystem, DoubleSupplier xController, DoubleSupplier yController, 
      DoubleSupplier rotationController, BooleanSupplier robotOriented) {
    this.swerveSubsystem = swerveSubsystem;
    this.xController = xController;
    this.yController = yController;
    this.rotationController = rotationController;

    this.robotOriented = robotOriented;

    rotationPositionPID.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double xSpeed = xController.getAsDouble();
    double ySpeed = yController.getAsDouble();

    double rotationSpeed = rotationController.getAsDouble();

    if (robotOriented.getAsBoolean()) {
      swerveSpeeds.vxMetersPerSecond = xSpeed;
      swerveSpeeds.vyMetersPerSecond = ySpeed;
      swerveSpeeds.omegaRadiansPerSecond = rotationSpeed;
    }
    else {
      swerveSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed, 
          ySpeed, 
          rotationSpeed, 
          Rotation2d.fromRadians(swerveSubsystem.getYaw()));
    }

    swerveSubsystem.setChassisSpeeds(swerveSpeeds);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
