package edu.mit.chip.utils;

public abstract class ChipCommand {
    protected State state;
    private boolean shouldTerminate;

    protected enum State {
        INITIALIZING,
        EXECUTING,
        ENDING,
        DONE;
    }

    public ChipCommand() {
        state = State.INITIALIZING;
        shouldTerminate = false;
    }

    public void terminate() {
        shouldTerminate = true;
    }

    public void tick() {
        switch (state) {
            case INITIALIZING:
                initialize();
                state = State.EXECUTING;
                break;
            case EXECUTING:
                execute();
                if (isFinished() || shouldTerminate)
                    state = State.ENDING;
                break;
            case ENDING:
                end();
                state = State.DONE;
                break;
            case DONE:
                break;
        }
    }

    protected abstract void initialize();
    protected abstract void execute();
    protected abstract boolean isFinished();
    protected abstract void end();
}