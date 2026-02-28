package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.List;

public class AprilTagLimelight {
    public Limelight3A limelight;

    public AprilTagLimelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // Default to pipeline 0
        limelight.start();
    }

    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    public void setPipeline(int index) {
        limelight.pipelineSwitch(index);
    }

    public LLResultTypes.FiducialResult getFiducialById(int id) {
        LLResult result = getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult f : fiducials) {
                if (f.getFiducialId() == id) return f;
            }
        }
        return null;
    }
}
