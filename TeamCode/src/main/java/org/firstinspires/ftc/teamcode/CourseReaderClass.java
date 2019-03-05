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
    File defaultFile = new File("./default_path.txt");

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
    private List<Double> AngleMap = new ArrayList<Double>();
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
                if(charBox.get(i)[j] == 'X') {
                    StartRow = i;
                    StartColumn = j;
                }
            }
            //System.out.println(charBox.get(i));
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
        int HasFoundPath = -1;
        int[] aux = {0, 0};

        for(int i = -1; i < 2; i++){
            for(int j = -1; j < 2; j++){
                if(InitPointLoc[0] + i < TextBlockHeight && InitPointLoc[0] + i >= 0 && InitPointLoc[1] + j < TextBlockWidth && InitPointLoc[1] + j >= 0) {
                    if(charBox.get(InitPointLoc[0] + i)[InitPointLoc[1] + j] == '1' && HasFoundPath < 2){
                        aux[0] = InitPointLoc[0] + i;
                        aux[1] = InitPointLoc[1] + j;
                        HasFoundPath = 1;
                    }
                    if (charBox.get(InitPointLoc[0] + i)[InitPointLoc[1] + j] == '0' && HasFoundPath < 1) {
                        aux[0] = InitPointLoc[0] + i;
                        aux[1] = InitPointLoc[1] + j;
                        HasFoundPath = 0;
                    }
                }
            }
        }

        if(HasFoundPath > -1) {
            System.out.println(charBox.get(aux[0])[aux[1]]);
            if(charBox.get(aux[0])[aux[1]] == '0') {
                charBox.get(aux[0])[aux[1]] = '*';
            }
            else {
                charBox.get(aux[0])[aux[1]] = '0';
            }
            AngleMap.add(CategorizeResults(aux[0] - InitPointLoc[0], aux[1] - InitPointLoc[1]));
            return aux;
        }
        else{
            return null;
        }
    }

    private double CategorizeResults(int row, int column){
        double desiredAngle = 0;

        if(Math.abs(row * column) == 1){
            desiredAngle = Math.toDegrees(Math.atan(WidthVector/HeightVector));
            if(row == 1){
                if(column == 1)
                    desiredAngle += 180;
                else if(column == -1)
                    desiredAngle = 180 - desiredAngle;
            }
            else if(row == -1 && column == 1){
                desiredAngle = 360 - desiredAngle;
            }
            VectorMap.add(Math.sqrt(Math.pow(HeightVector, 2) + Math.pow(WidthVector, 2)));
        }
        else if(row == 0 && column == 0){
            VectorMap.add(0.0);
        }
        else if(row == 0){
            desiredAngle = 180 + (column * 90);
            VectorMap.add(WidthVector);
        }
        else if(column == 0){
            desiredAngle = 90 + (row * 90);
            VectorMap.add(HeightVector);
        }
        return desiredAngle;
    }

    public List<Double> GetAngleMap(){
        return AngleMap;
    }

    public List<Double> GetVectorMap(){
        return VectorMap;
    }

}





