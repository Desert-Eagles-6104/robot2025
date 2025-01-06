package frc.DELib25.CSV;

import java.io.FileWriter;
import java.io.IOException;

public class CSVWriter {
    private String m_filePath;

    public CSVWriter(String filePath) {
        this.m_filePath = filePath;
    }

    public void writeCSVFile(double[][] dataArray) {
        try (FileWriter writer = new FileWriter(m_filePath)) {
            for (double[] row : dataArray) {
                for (int i = 0; i < row.length; i++) {
                    writer.append(String.valueOf(row[i]));

                    if (i < row.length - 1) {
                        writer.append(",");
                    } else {
                        writer.append("\n");
                    }
                }
            }

            System.out.println("CSV file written successfully at: " + m_filePath);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
