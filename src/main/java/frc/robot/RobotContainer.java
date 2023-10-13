// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final Drivetrain drivetrain;

    // The driver's controller
    CommandXboxController driver;
    CommandXboxController operator;

    SendableChooser<Command> autoSelector;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        drivetrain = new Drivetrain();
        driver = new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);
        operator = new CommandXboxController(OIConstants.OPERATOR_CONTROLLER_PORT);
        autoSelector = new SendableChooser<>();

        
        // Configure default commands
        // Set the default drive command to split-stick arcade drive
        drivetrain.setDefaultCommand(
            // A split-stick arcade command, with forward/backward controlled by the left
            // hand, and turning controlled by the right.
            new RunCommand(
                () -> drivetrain.drive(
                    MathUtil.applyDeadband(driver.getLeftY(), OIConstants.DRIVER_DEADZONE), 
                    MathUtil.applyDeadband(driver.getRightX(), OIConstants.DRIVER_DEADZONE)),
                    drivetrain));
                    
        // Configure the button bindings
        configureButtonBindings();

        // Add commands to the autonomous command chooser
        addAutos();
    }

    private void addAutos(){
        autoSelector.addOption("DEFAULT", drivetrain.run(() -> drivetrain.drive(1, 0)).withTimeout(2).asProxy());
        SmartDashboard.putData(autoSelector);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        driver.leftStick().toggleOnTrue(
            Commands.run(() -> drivetrain.setMaxOutput(0.5))
                .finallyDo((end) -> drivetrain.setMaxOutput(1)));
    }

    public void onEnabled() {
        drivetrain.resetRelativeRotationEncoders();
    }

    public void onDisabled() {
        drivetrain.scheduleConfigCommands();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoSelector.getSelected();
    }
}