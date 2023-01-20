package org.firstinspires.ftc.teamcode.hardware;

import java.util.ArrayList;

public class AutonomousSystem {

    public int countMove;
    public int currMove;
    public boolean justInc = true;

    public ArrayList<AutonomousSystem> subsystem = new ArrayList<>();

    public boolean next(AutonomousSystem target) {

        boolean rightMove = countMove == currMove;

        if (!justInc && rightMove && target.isDone()) {

            currMove++;
            justInc = true;
        }

        countMove++;

        return rightMove;
    }

    public boolean next() {

        return next(this);
    }

    public void update() {

        countMove = 0;
        justInc = false;
        for (AutonomousSystem sub : subsystem) sub.update();
    }

    public boolean isDone() {

        boolean done = true;
        for (AutonomousSystem sub : subsystem) done = done && sub.isDone();
        return done;
    }
}
