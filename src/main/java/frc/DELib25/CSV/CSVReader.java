package frc.DELib25.CSV;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Scanner;

public class CSVReader {
    
    private final static String ITEM_SEPARATOR = ",";

    private final Scanner reader;
    private final int lines;

    public CSVReader(String filePath) throws IOException {
        Path path = Paths.get(filePath);
        File file = new File(filePath);

        if(!file.exists()) {
            createEmptyFile(filePath);
        }

        this.lines = (int)Files.lines(path).count();
        this.reader = new Scanner(file);
    }

    public double[][] readAsDouble(int columns)  { 
        double array[][] = new double[this.lines][columns];
        for(int i = 0; i < this.lines; i++) {
            String[] row = this.reader.nextLine().split(ITEM_SEPARATOR);
            for(int j = 0; j < columns; j++) {
                array[i][j] = Double.parseDouble(row[j].replaceAll("\\s", ""));
            }
        }

        this.reader.close();
        return array;
    }

    private void createEmptyFile(String filePath) {
        try (FileWriter writer = new FileWriter(filePath)) {
            // Write an empty line to the file
            writer.write("");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}