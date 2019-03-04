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
    File defaultFile = new File("C:\\Users\\robos\\IdeaProjects\\TestChestieRobo\\default_test.txt");

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
    private List<char[]> charBox = new ArrayList<char[]>();
    //Text bounds
    private static double TextBlockHeight = 0;
    private static double TextBlockWidth = 0;
    //Start coordinates
    private int StartRow, StartColumn;
    //Instruction map
    private List<Integer> AngleMap = new ArrayList<Integer>();
    private List<Double> VectorMap = new ArrayList<Double>();


    //Constructor
    CourseReaderClass(File inFile) throws Exception, NullPointerException {

        if(!inFile.canExecute())
            inFile = defaultFile;

        BufferedReader br = new BufferedReader(new FileReader(inFile));

        String st;
        int index = 0;
        while ((st = br.readLine()) != null) {
            charBox.add(st.toCharArray());
            TextBlockHeight++;
            if(TextBlockWidth <= st.length())
                TextBlockWidth = st.length();
            index++;
        }

        //Interpret Map
        //Find the starting position
        for(int i = 0; i < charBox.size(); i++){
            for(int j = 0; j < charBox.get(i).length; j++){
                if(charBox.get(i)[j] == 'X')
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
                if(InitPointLoc[0] + i < TextBlockHeight && InitPointLoc[0] + i >= 0 && InitPointLoc[1] + j < TextBlockWidth && InitPointLoc[1] + j >= 0) {
                    if (charBox.get(InitPointLoc[0] + i)[InitPointLoc[1] + j] == '0') {
                        int[] aux = {InitPointLoc[0] + i, InitPointLoc[1] + j};
                        charBox.get(InitPointLoc[0] + i)[InitPointLoc[1] + j] = '*';
                        AngleMap.add(CategorizeResults(i, j));
                        return aux;
                    }
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
                        VectorMap.add(Math.sqrt(Math.pow(HeightVector, 2) + Math.pow(WidthVector, 2)));
                        break;
                    case 0:
                        desiredAngle = 0;
                        VectorMap.add(HeightVector);
                        break;
                    case 1:
                        desiredAngle = 315;
                        VectorMap.add(Math.sqrt(Math.pow(HeightVector, 2) + Math.pow(WidthVector, 2)));
                        break;
                }
                break;
            case 0:
                switch (column){
                    case -1:
                        desiredAngle = 90;
                        VectorMap.add(WidthVector);
                        break;
                    case 1:
                        desiredAngle = 270;
                        VectorMap.add(WidthVector);
                        break;
                }
                break;
            case 1:
                switch (column){
                    case -1:
                        desiredAngle = 135;
                        VectorMap.add(Math.sqrt(Math.pow(HeightVector, 2) + Math.pow(WidthVector, 2)));
                        break;
                    case 0:
                        desiredAngle = 180;
                        VectorMap.add(HeightVector);
                        break;
                    case 1:
                        desiredAngle = 225;
                        VectorMap.add(Math.sqrt(Math.pow(HeightVector, 2) + Math.pow(WidthVector, 2)));
                        break;
                }
                break;
        }

        return desiredAngle;
    }

    public List<Integer> GetAngleMap(){
        return AngleMap;
    }

    public List<Double> GetVectorMap(){
        return VectorMap;
    }

}


