package utility;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import subsystems.Intake;
import subsystems.Spindexer;
import utility.RobotConstants.Enums.CatalogingCases;

import static utility.SpindexerAndMotifStatus.SpindexerPattern.setBallInSlotX;

public class CatalogManager {
   private final Intake intake;
    private final Telemetry telemetry;
    private final DualBallDetector sensors;
    private final Spindexer spindexer;
    private int indexed = 0;

    private CatalogingCases state = RobotConstants.Enums.CatalogingCases.Idle;;

    // Timeout and retry management
    private ElapsedTime stateTimer = new ElapsedTime();
    private int rotationRetries = 0;
    private boolean isRetryingRotation = false;


    public CatalogManager(Spindexer spindexer, Intake intake, Telemetry telemetry, DualBallDetector sensors) {
        this.spindexer = spindexer;
        this.telemetry = telemetry;
        this.intake = intake;
        this.sensors = sensors;
    }

    public CatalogingCases getState(){
        return state;
    }


    public void update() {
        sensors.update();
        switch (state) {
            case Idle:
                return;
            case Scanning:
                // Check for timeout - no ball detected for too long
                if (stateTimer.seconds() > RobotConstants.Cataloging.SCAN_TIMEOUT_SECONDS) {
                    // Timeout: finish with whatever balls we have indexed
                    state = CatalogingCases.RotateToEndLocation;
                    break;
                }

                DualBallDetector.Result result = sensors.detectBall();
                if (result.ballPresent) {
                    indexed++;
                    setBallInSlotX(0, result.color);
                    if(indexed == 3){
                        state = CatalogingCases.RotateToEndLocation;
                        break;
                    }else{
                        state = CatalogingCases.WaitingForRotation;
                        stateTimer.reset(); // Reset timer for rotation timeout
                        intake.runIntake();
                        spindexer.rotateCCW();
                    }
                }
                break;
            case WaitingForRotation:
                // Check for rotation timeout - spindexer might be stuck
                if (stateTimer.seconds() > RobotConstants.Cataloging.ROTATION_TIMEOUT_SECONDS) {
                    if (rotationRetries < RobotConstants.Cataloging.MAX_ROTATION_RETRIES) {
                        // Attempt retry: rotate opposite direction to unstick
                        state = CatalogingCases.RetryRotation;
                        rotationRetries++;
                        stateTimer.reset();
                        break;
                    } else {
                        // Retries exhausted: give up and finish with what we have
                        state = CatalogingCases.RotateToEndLocation;
                        break;
                    }
                }

                if(spindexer.isRotationIdle()){
                    intake.stopIntake();
                    state = CatalogingCases.Scanning;
                    stateTimer.reset(); // Reset timer for next scan
                }
                break;
            case RetryRotation:
                // Two-phase retry: first rotate opposite direction (unstick), then retry original direction
                if (!isRetryingRotation) {
                    // Phase 1: Rotate opposite direction (CW) to unstick
                    spindexer.rotateCW();
                    stateTimer.reset();
                    isRetryingRotation = true;
                } else if (spindexer.isRotationIdle()) {
                    // Phase 2: Rotate back to correct direction (CCW)
                    spindexer.rotateCCW();
                    isRetryingRotation = false;
                    stateTimer.reset();
                    state = CatalogingCases.WaitingForRotation; // Go back to waiting for rotation
                }
                break;
            case RotateToEndLocation:
                state = CatalogingCases.Idle;
                intake.stopIntake();
                indexed = 0;
                rotationRetries = 0;
                isRetryingRotation = false;
                spindexer.rotateToColor(SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(0));
                break;
        }
    }


    public void initiateCataloging(){
        if(state != CatalogingCases.Idle){
            return;
        }
        state = CatalogingCases.Scanning;
        indexed = 0;
        stateTimer.reset();
        rotationRetries = 0;
        isRetryingRotation = false;
    }










}
