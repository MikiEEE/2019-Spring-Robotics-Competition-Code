
import java.util.*;


public class AStar {

    public static final int DIAGONAL_COST = 14;
    public static final int V_H_COST = 10;

    //Blocked cells are just null Cell values in grid
    static Cell [][] grid = new Cell[5][5];

    static LinkedList<Cell> open;

    static boolean closed[][];
    static int startI, startJ;
    static int endI, endJ;

    public static void setBlocked(int i, int j){
        grid[i][j] = null;
    }

    public static void setStartCell(int i, int j){
        startI = i;
        startJ = j;
    }

    public static void setEndCell(int i, int j){
        endI = i;
        endJ = j;
    }

    static void checkAndUpdateCost(Cell current, Cell t, int cost){
        if(t == null || closed[t.i][t.j])return;
        int t_final_cost = t.heuristicCost+cost;

        boolean inOpen = open.contains(t);
        if(!inOpen || t_final_cost<t.finalCost){
            t.finalCost = t_final_cost;
            t.parent = current;
            if(!inOpen){
              open.add(t);
              Collections.sort(open,new Comp());
            }
        }
    }

    public static void AStar(){

        //add the start location to open list.
        open.add(grid[startI][startJ]);

        Cell current;

        while(true){
            current = open.poll();
            if(current==null)break;
            closed[current.i][current.j]=true;

            if(current.equals(grid[endI][endJ])){
                return;
            }

            Cell t;
            if(current.i-1>=0){
                t = grid[current.i-1][current.j];
                checkAndUpdateCost(current, t, current.finalCost+V_H_COST);
            }

            if(current.j-1>=0){
                t = grid[current.i][current.j-1];
                checkAndUpdateCost(current, t, current.finalCost+V_H_COST);
            }

            if(current.j+1<grid[0].length){
                t = grid[current.i][current.j+1];
                checkAndUpdateCost(current, t, current.finalCost+V_H_COST);
            }

            if(current.i+1<grid.length){
                t = grid[current.i+1][current.j];
                checkAndUpdateCost(current, t, current.finalCost+V_H_COST);
            }
        }
    }

    /*
    Params :
    x, y = Board's dimensions
    si, sj = start location's x and y coordinates
    ei, ej = end location's x and y coordinates
    int[][] blocked = array containing inaccessible cell coordinates
    */
    public static LinkedList<Cell> getQueue(int x, int y, int si, int sj, int ei, int ej, int[][] blocked) {
       //Reset
      grid = new Cell[x][y];
      closed = new boolean[x][y];
      open = new LinkedList<Cell>();



      //Set start position
      setStartCell(si, sj);  //Setting to 0,0 by default. Will be useful for the UI part

      //Set End Location
      setEndCell(ei, ej);

      for(int i=0;i<x;++i){
         for(int j=0;j<y;++j){
             grid[i][j] = new Cell(i, j);
             grid[i][j].heuristicCost = Math.abs(i-endI)+Math.abs(j-endJ);
         }
      }
      grid[si][sj].finalCost = 0;

      /*
        Set blocked cells. Simply set the cell values to null
        for blocked cells.
      */
      for(int i=0;i<blocked.length;++i){
          setBlocked(blocked[i][0], blocked[i][1]);
      }

      AStar();

      LinkedList<Cell> result = new LinkedList<Cell>(); // Result to be returned

      if(closed[endI][endJ]){
          //Trace back the path
           Cell current = grid[endI][endJ];
           result.add(current);
           while(current.parent!=null){
               result.add(current.parent);
               current = current.parent;
           }
      }else result = null;
      return result;
    }
}
