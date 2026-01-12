package frc.robot;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.DriveCommands;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.Subsystems.Climber.*;
import frc.robot.Subsystems.Drive.*;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  // Chassis
  private final Drive m_driveSubsystem;

  private final Climber m_climberSubsystem;

  // Controllers
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER);
  private final CommandXboxController m_auxButtonBoard =
      new CommandXboxController(OperatorConstants.AUX_BUTTON_BOARD);
  private final CommandXboxController m_auxController =
      new CommandXboxController(OperatorConstants.AUX_XBOX_CONTROLLER);


  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    switch (RobotStateConstants.getMode()) {
        // Real robot, instantiates hardware IO implementations
      case REAL:
        m_driveSubsystem =
            new Drive(
                new ModuleIOSparkMaxTalonFX(0),
                new ModuleIOSparkMaxTalonFX(1),
                new ModuleIOSparkMaxTalonFX(2),
                new ModuleIOSparkMaxTalonFX(3),
                new GyroIOPigeon2());
        m_climberSubsystem = new Climber(new ClimberIOTalonFX());
        break;
        // Sim robot, instantiates physics sim IO implementations
      case SIM:
        m_driveSubsystem =
            new Drive(
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new GyroIO() {});
        m_climberSubsystem = new Climber(new ClimberIOSim());
        break;
        // Replayed robot, disables all IO implementations
      default:
        m_driveSubsystem =
            new Drive(
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new GyroIO() {});
        m_climberSubsystem = new Climber(new ClimberIO() {});
        break;
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    this.driverControllerBindings();
    this.auxButtonBoardBindings();
  }

  /** Driver Controls */
  private void driverControllerBindings() {
    /* Driving the robot */
    // Default to field relative driving
    m_driveSubsystem.setDefaultCommand(
        DriveCommands.fieldRelativeDrive(
                m_driveSubsystem,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                () -> 0.8 * -m_driverController.getRightX())
            .withName("FieldRelativeDrive"));
  }

  /** Aux Button Board Controls */
  public void auxButtonBoardBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }

  /**
   * Sets all mechanisms to brake mode, intended for use when the robot is disabled.
   *
   * @param enable {@code true} to enable brake mode, {@code false} to enable coast mode.
   */
  public void allMechanismsBrakeMode(boolean enable) {
    m_driveSubsystem.enableBrakeModeAll(enable);
    // m_algaePivotSubsystem.enableBrakeMode(enable);
    // m_periscopeSubsystem.enableBrakeMode(enable);
    m_climberSubsystem.enableBrakeMode(enable);
  }
}
