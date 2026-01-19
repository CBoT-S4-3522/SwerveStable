package frc.robot;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.SwerveBase;

public class RobotContainer {
    /*  Shuffleboard */
    public static ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");

    /* Controllers */
    private final Joystick driver1 = new Joystick(Constants.OIConstants.kDriver1Port);

    /* Subsystems */
    private final SwerveBase s_Swerve;

/////Driver 1////////////
    private final int translationX = XboxController.Axis.kLeftY.value;
    private final int translationY = XboxController.Axis.kLeftX.value;
    private final int rotation = XboxController.Axis.kRightX.value;

    private final JoystickButton zeroGyro = new JoystickButton(driver1, XboxController.Button.kRightStick.value);
    private final DoubleSupplier turbo = () -> driver1.getRawAxis(2);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
    s_Swerve = new SwerveBase();
    //Autos
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Giro", autoChooser);
    SmartDashboard.putData("Frente", autoChooser);
    SmartDashboard.putData("Derecha", autoChooser);
    SmartDashboard.putData("Prueba", autoChooser);

    /* Swerve */
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve,
                () -> -driver1.getRawAxis(translationX),
                () -> -driver1.getRawAxis(translationY),
                () -> driver1.getRawAxis(rotation),
                turbo,
                () -> driver1.getRawButtonPressed(XboxController.Button.kLeftBumper.value)
            )
        );

    // Configure the button bindings
        configureButtonBindings();
    }
    private void configureButtonBindings() {


    //Reset Gyro
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

    }
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
