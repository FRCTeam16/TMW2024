package frc.robot.subsystems.vision;

public enum Pipeline {
    April(0),
    Note(1);


    public final int pipelineNumber;

    private Pipeline(int number) {
        this.pipelineNumber = number;
    }

    public static Pipeline findPipeline(int value) {
        for (var pipeline : Pipeline.values()) {
            if (pipeline.pipelineNumber == value) {
                return pipeline;
            }
        }
        return null;
    }
}
