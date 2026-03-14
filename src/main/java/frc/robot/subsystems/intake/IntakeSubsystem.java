package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.DataProcessor;
import frc.robot.subsystems.intake.pivot.IntakePivotIO;
import frc.robot.subsystems.intake.pivot.IntakePivotIOInputsAutoLogged;
import frc.robot.subsystems.intake.rollers.IntakeRollersIO;
import frc.robot.subsystems.intake.rollers.IntakeRollersIOInputsAutoLogged;

public class IntakeSubsystem extends SubsystemBase {
  private final IntakePivotIO pivotIO;
  private final IntakeRollersIO rollersIO;

  private final IntakePivotIOInputsAutoLogged pivotInputs = new IntakePivotIOInputsAutoLogged();
  private final IntakeRollersIOInputsAutoLogged rollersInputs =
      new IntakeRollersIOInputsAutoLogged();

  public IntakeSubsystem(IntakePivotIO pivotIO, IntakeRollersIO rollersIO) {
    this.pivotIO = pivotIO;
    this.rollersIO = rollersIO;

    DataProcessor.initDataProcessor(
        () -> {
          synchronized (pivotInputs) {
            synchronized (rollersInputs) {
              pivotIO.updateInputs(pivotInputs);
              rollersIO.updateInputs(rollersInputs);
            }
          }
        },
        pivotIO,
        rollersIO);
  }
}
