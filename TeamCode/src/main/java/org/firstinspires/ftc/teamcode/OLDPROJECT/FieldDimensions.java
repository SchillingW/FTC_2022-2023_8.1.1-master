package org.firstinspires.ftc.teamcode.OLDPROJECT;

public class FieldDimensions {

    public double botanchor2botcenterHARDWARE;
    public double botanchor2clawcenterHARDWARE;

    public double cellcorner2botanchorPLACEMENT;
    public double cellPLACEMENT;

    public double cellLength = 23.5;

    public double toCell(double i) {

        return (i + 0.5 - cellPLACEMENT) * cellLength - botanchor2botcenterHARDWARE - cellcorner2botanchorPLACEMENT;
    }

    public double toPole(double i) {

        return (i + 1 - cellPLACEMENT) * cellLength - botanchor2clawcenterHARDWARE - cellcorner2botanchorPLACEMENT;
    }

    public double toCone(double i) {

        return (i + 0.5 - cellPLACEMENT) * cellLength - botanchor2clawcenterHARDWARE - cellcorner2botanchorPLACEMENT;
    }
}
