package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class IntakeIOSim implements IntakeIO {
  private final IntakeSimulation intakeSimulation;

  public IntakeIOSim(AbstractDriveTrainSimulation driveTrain) {
    this.intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            // Este es el tipo de objeto que el intake va a recoger
            "Fuel",
            // Specify the drivetrain to which this intake is attached
            driveTrain,
            // The width of the intake (CHANGEEEEE)
            Meters.of(0.508), // 20 inches
            // The extension length of the intake beyond the robot's frame (when activated)
            Meters.of(0.2),
            // The intake is mounted on the front? side of the chassis
            IntakeSimulation.IntakeSide.FRONT,
            // The intake can hold up to ? balls?
            100);
  }

  public void setRunning(boolean runIntake) {
    if (runIntake)
      intakeSimulation
          .startIntake(); // Extends the intake out from the chassis frame and starts detecting
    // contacts with game pieces
    else
      intakeSimulation
          .stopIntake(); // Retracts the intake into the chassis frame, disabling game piece
    // collection
  }

  public boolean isAnyFuelInsideIntake() {
    return intakeSimulation.getGamePiecesAmount()
        != 0; // True if there is a game piece in the intake
  }

  public void launchFuel() {
    // if there is a fuel in the intake, it will be removed and return true; otherwise, returns
    // false
    if (intakeSimulation.obtainGamePieceFromIntake()) {}
    // ShooterIOSim.launchNote(); //notify the simulated flywheels to launch a fuel (We need the
    // shooter to be aware of this)
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {}

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public void stopRollers() {}
}
