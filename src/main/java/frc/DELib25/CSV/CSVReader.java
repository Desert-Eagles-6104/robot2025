package frc.DELib25.CSV;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Scanner;

public class CSVReader {
    //#region Constants 
    private final static String k_itemSeparator = ",";
    //#endregion

    //#region Private Members
    private final Scanner m_reader;
    private final int m_lines;
    //#endregion

    //#region Constructors
    public CSVReader(String filePath) throws IOException {
        Path path = Paths.get(filePath);
        File file = new File(filePath);

        if(!file.exists()) {
            createEmptyFile(filePath);
        }

        m_lines = (int)Files.lines(path).count();
        m_reader = new Scanner(file);
    }
    //#endregion

    //#region Public Methods
    public double[][] readAsDouble(int columns)  { 
        double array[][] = new double[m_lines][columns];
        for(int i = 0; i < m_lines; i++) {
            String[] row = m_reader.nextLine().split(k_itemSeparator);
            for(int j = 0; j < columns; j++) {
                array[i][j] = Double.parseDouble(row[j].replaceAll("\\s", ""));
            }
        }

        m_reader.close();
        return array;
    }
    //#endregion

    private void createEmptyFile(String filePath) {
        try (FileWriter writer = new FileWriter(filePath)) {
            // Write an empty line to the file
            writer.write("");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}