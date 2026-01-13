package utility;

import Constants.EnumConstants.CatalogingCases;
import Constants.RobotConstants;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import subsystems.Intake;
import subsystems.Spindexer;

import static utility.SpindexerAndMotifStatus.SpindexerPattern.setBallInSlotX;

public class CatalogManager {
   private final Intake intake;
    private final Telemetry telemetry;
    private final DualBallDetector sensors;
    private final Spindexer spindexer;
    private int indexed = 0;

    private CatalogingCases state = CatalogingCases.Idle;

    // Timeout management
    private ElapsedTime stateTimer = new ElapsedTime();


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
                    // Timeout: finish with whatever balls we have indexed
                    state = CatalogingCases.RotateToEndLocation;
                    break;
                }

                if(spindexer.isRotationIdle()){
                    intake.stopIntake();
                    state = CatalogingCases.Scanning;
                    stateTimer.reset(); // Reset timer for next scan
                }
                break;
            case RotateToEndLocation:
                state = CatalogingCases.Idle;
                intake.stopIntake();
                indexed = 0;
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
    }










}
