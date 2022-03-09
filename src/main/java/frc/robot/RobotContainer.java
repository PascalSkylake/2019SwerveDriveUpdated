// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveSwerveWithXbox;
import frc.robot.commands.HoldAngle;
import frc.robot.commands.RotateToTargetWhileDriving;
import frc.robot.subsystems.CPManipulator;
import frc.robot.subsystems.SwerveDrive;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.links.SPILink;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
    // The robot's subsystems and commands are defined here...
    public static final Pixy2 pixy = Pixy2.createInstance(new SPILink());

    public static AHRS navx = new AHRS();
    public static SwerveDrive swerveDrive = new SwerveDrive();
    public static XboxController xboxController = new XboxController(RobotMap.XBOX_PORT);

    public static DriveSwerveWithXbox driveSwerveWithXbox = new DriveSwerveWithXbox();
    private RotateToTargetWhileDriving rotateToTarget = new RotateToTargetWhileDriving();
    public static HoldAngle holdAngle = new HoldAngle();

    public static final CPManipulator cpanel = new CPManipulator();

    JoystickButton rotateToAngle;
    CustomButton turnAndDrive;
    
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        // Configure the button bindings
        pixy.init();
        configureButtonBindings();
    }
    
    
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings()
    {
        // Add button to command mappings here.
        // See https://docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html
        rotateToAngle = new JoystickButton(xboxController, XboxController.Button.kY.value);
        rotateToAngle.whenHeld(rotateToTarget);

        turnAndDrive = new CustomButton( () -> Math.abs(xboxController.getRightX()) > 0.05 );
        turnAndDrive.whenHeld(driveSwerveWithXbox);
    }
    
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        // An ExampleCommand will run in autonomous
        return null;
    }
}
