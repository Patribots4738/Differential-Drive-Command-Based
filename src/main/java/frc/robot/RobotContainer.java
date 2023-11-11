// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
    //TODO: Make a Field that update the pose of the robot
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        drivetrain = new Drivetrain();
        driver = new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);
        operator = new CommandXboxController(OIConstants.OPERATOR_CONTROLLER_PORT);
        autoSelector = new SendableChooser<>();

        /*
         * Configure the default command for the drivetrain subsystem, this is what
         * will be run when no other commands are running that require the drivetrain.
         */
        //TODO: Change the default command to a RunCommand that drives the robot using arcade drive style
        /*
         * This Run Command will drive our robot using arcade drive style, i.e. using
         * the left stick to control forward/backward movement and the right stick to
         * control turning.
         * 
         * <p>
         * The Run Command is a command that will be scheduled through the 
         * Command Scheduler and will run until it is canceled. 
         */
        drivetrain.setDefaultCommand(new RunCommand(() -> {}));
                    
        /*
         * Configure the button bindings,
         * this is where you choose which commands are bound to which controller
         * buttons/axis, etc... 
         * 
         * <p>
         * This is one of the main points of the Command Based framework,
         * it allows you to easily change which commands are bound to which
         * buttons/axis, without changing the commands themselves.
         */
        configureButtonBindings();

        /*
         * add the autonomous commands to the auto selector, to then choose from
         * on the smart dashboard
         */
        addAutos();
    }

    /**
     * This the the periodic method that is called every 20ms when the robot is
     * enabled.
     */
    public void periodic() {}

    /**
     * Add the autonomous commands to the auto selector, to then choose from
     * on the smart dashboard
     * 
     * <p>
     * This is where you add your autonomous commands to the auto selector
     * (A default command is already added for you that drives 
     * forward for 2 seconds at max speed)
     */
    private void addAutos(){
        autoSelector.addOption("DEFAULT", drivetrain.run(() -> drivetrain.drive(1, 0)).withTimeout(2).asProxy());
        SmartDashboard.putData(autoSelector);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link CommandXboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        // TODO: On click of the left stick, toggle the max speed of the robot between 50% and 100%
    }

    /*
     * The onEnabled method is called when the robot is enabled,
     * and is used to reset sensors, etc...
     */
    public void onEnabled() {
        drivetrain.resetRelativeRotationEncoders();
    }

    /*
     * The onDisabled method is called when the robot is disabled,
     * and is used to config our motors for our drive, etc...
     */
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