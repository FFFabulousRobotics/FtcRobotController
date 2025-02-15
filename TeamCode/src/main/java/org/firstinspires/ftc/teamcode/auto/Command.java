package org.firstinspires.ftc.teamcode.auto;

public interface Command {
    /**
     * This method is called in the main loop of OpMode
     * It should contain the code of EACH TICK, instead of filterGain loop in it!
     */
    void iterate();

    /**
     * To judge whether the command needs to continue.
     * @return Return true when it need to continue, otherwise false.
     */
    boolean hasNext();

    /**
     * This method is called once when the command starts looping in the OpMode.
     */
    default void init() {
    }

    /**
     * The method is called when the command is finished.
     */
    default void finish() {
    }

    /**
     * The method is called when the command is canceled.
     */
    default void cancel() {
    }
}
