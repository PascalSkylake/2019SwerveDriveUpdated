/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Limelight;
import frc.robot.Limelight.LightMode;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive;

public class RotateToTargetWhileDriving extends CommandBase {
  private double angle;
  private boolean isFieldOriented;

  /**
   * Creates a new RotateToAngle.
   */
  public RotateToTargetWhileDriving() {
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.swerveDrive.resetPid();
    Limelight.setLedMode(LightMode.eOn);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angle = -RobotContainer.navx.getYaw() - Limelight.getTx();

    var xSpeed = -RobotContainer.xboxController.getLeftY() * SwerveDrive.kMaxSpeed;

    var ySpeed = -RobotContainer.xboxController.getLeftX() * SwerveDrive.kMaxSpeed;
    
    isFieldOriented = (!RobotContainer.xboxController.getLeftBumper());

    RobotContainer.swerveDrive.holdAngleWhileDriving(xSpeed, ySpeed, angle, isFieldOriented);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Limelight.setLedMode(LightMode.eOff); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
