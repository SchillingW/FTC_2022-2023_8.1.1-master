package org.firstinspires.ftc.teamcode.hardware;

import java.util.ArrayList;

public class AutonomousSystem {

    // keep track of move order in autonomous
    public int countMove;
    public int currMove;

    // true when move incremented in current frame
    public boolean justInc = true;

    // list of subsystems in this system
    public ArrayList<AutonomousSystem> subsystem = new ArrayList<>();

    // return true if program has reaches this move & progress to next move if all systems are done
    public boolean next(AutonomousSystem... target) {

        // is this the current move
        boolean rightMove = countMove == currMove;

        // are all subsystems done
        boolean done = true;
        for (AutonomousSystem sub : target) done = done && sub.isDone();

        // if done then increment to next move
        if (!justInc && rightMove && done) {

            currMove++;
            justInc = true;
        }

        // increment call counter within frame
        countMove++;

        // return true if on this move
        return rightMove;
    }

    // return true if previously checked move just finished
    public boolean end() {

        return justInc && countMove == currMove;
    }

    public void update() {

        // reset move order counts
        countMove = 0;
        justInc = false;

        // update all subsystems
        for (AutonomousSystem sub : subsystem) sub.update();
    }

    public boolean isDone() {

        // return true if all subsystems are done
        boolean done = true;
        for (AutonomousSystem sub : subsystem) done = done && sub.isDone();
        return done;
    }
}
