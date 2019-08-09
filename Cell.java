package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import java.util.*;

public class Cell {
    int heuristicCost = 0; //Heuristic cost
    int finalCost = 0; //G+H
    int i, j;
    Cell parent;
    Hashtable<String, Integer> coordinates = new Hashtable<String, Integer>();

    Cell(int i, int j){
        this.i = i;
        this.j = j;
        coordinates.put("X",i);
        coordinates.put("Y",j);
    }

    public Hashtable<String,Integer> getCoordinates() {
      return coordinates;
    }

    public int getFinalCost() {
      return this.finalCost;
    }

    @Override
    public String toString(){
        return "["+this.i+", "+this.j+"]";
    }
}
