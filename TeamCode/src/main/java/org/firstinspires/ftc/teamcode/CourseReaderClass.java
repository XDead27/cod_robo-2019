package org.firstinspires.ftc.teamcode;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

public class CourseReaderClass {
    File defaultFile = new File("C:\\Users\\robos\\Documents\\Android_Studio_Apps\\cod_robo-2019\\default_test.txt");

    //RULES FOR ASCII FILE
    //
    // X - starting point
    // * - empty space
    // 0 - go once
    // 1 - go twice


    //Global Field Constants
    private static double Width = 358.14, Height = 358.14;
    private double WidthVector, HeightVector;

    //The ascii vector
    private char[][] charBox;
    //Text bounds
    private static double TextBlockHeight = 0;
    private static double TextBlockWidth = 0;
    //Start coordinates
    private int StartRow, StartColumn;
    //Instruction map
    private List<Integer> AngleMap = new ArrayList<Integer>();


    //Constructor
    CourseReaderClass(File inFile) throws IOException {

        if(!inFile.canExecute())
            inFile = defaultFile;

        BufferedReader br = new BufferedReader(new FileReader(inFile));

        String st;
        int index = 0;
        while ((st = br.readLine()) != null) {
            charBox[index] = st.toCharArray();
            TextBlockHeight++;
            if(TextBlockWidth <= st.length())
                TextBlockWidth = st.length();
            index++;
        }

        //Interpret Map
        //Find the starting position
        for(int i = 0; i < charBox.length; i++){
            for(int j = 0; j < charBox[i].length; j++){
                if(charBox[i][j] == 'X')
                    StartRow = i;
                    StartColumn = j;
            }
        }

        //Calculate vector lenght
        WidthVector = Width/TextBlockWidth;
        HeightVector = Height/TextBlockHeight;

        //Find the path
        int[] CurrentPosition = {StartRow, StartColumn};
        while(CurrentPosition != null){
            CurrentPosition = FindClosestToPoint(CurrentPosition);
        }

    }

    //Find the closest next path element
    private int[] FindClosestToPoint(int[] InitPointLoc){
        for(int i = -1; i < 2; i++){
            for(int j = -1; j < 2; j++){
                if(charBox[InitPointLoc[0] + i][InitPointLoc[1] + j] == '0') {
                    int[] aux = {InitPointLoc[0], InitPointLoc[1]};
                    charBox[InitPointLoc[0] + i][InitPointLoc[1] + j] = '*';
                    AngleMap.add(CategorizeResults(i, j));
                    return aux;
                }
            }
        }
        return null;
    }

    private int CategorizeResults(int row, int column){
        int desiredAngle = 0;
        switch (row){
            case -1:
                switch (column){
                    case -1:
                        desiredAngle = 45;
                        break;
                    case 0:
                        desiredAngle = 0;
                        break;
                    case 1:
                        desiredAngle = 315;
                        break;
                }
                break;
            case 0:
                switch (column){
                    case -1:
                        desiredAngle = 90;
                        break;
                    case 1:
                        desiredAngle = 270;
                        break;
                }
                break;
            case 1:
                switch (column){
                    case -1:
                        desiredAngle = 135;
                        break;
                    case 0:
                        desiredAngle = 180;
                        break;
                    case 1:
                        desiredAngle = 225;
                        break;
                }
                break;
        }

        return desiredAngle;
    }

    public List<Integer> GetAngleMap(){
        return AngleMap;
    }

    public double GetPlanarVector(){
        return Math.sqrt(Math.pow(HeightVector, 2) + Math.pow(WidthVector, 2));
    }

}


