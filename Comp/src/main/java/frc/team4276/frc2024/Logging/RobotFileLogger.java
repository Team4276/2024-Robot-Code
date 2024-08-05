package frc.team4276.frc2024.Logging;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import frc.team4276.frc2024.Constants.DebugConstants;

public class RobotFileLogger {
    private File logFile;
    private FileWriter writer;
    private String logPath;
    private static String logDirectory = DebugConstants.logDirectory;
    private boolean fileIsBeingAccessed;

    public RobotFileLogger(String fileName) {
        logPath = logDirectory + fileName;
        logFile = new File(logPath);
        try {
            logFile.createNewFile();
            writer = new FileWriter(logPath);
        } catch (IOException e) {
            PrintLogger.print("Exception thrown while creating file logger, file logging will not be available: " + "\n" + e.getMessage());
            writer = null;
        }
    }

    public synchronized void clearFile() {
        if (fileIsBeingAccessed) {
            try {
                wait();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }
        fileIsBeingAccessed = true;
        if (writer != null) {
            try {
                //deletes and recreates the file
                writer.close();
                logFile.delete();
                logFile.createNewFile();
                writer = new FileWriter(logPath);
            } catch (IOException e) {
                PrintLogger.print("Failed to clear file: " + "\n" + e.getMessage());
            }
        }
        fileIsBeingAccessed = false;
        notifyAll();
    }

    public synchronized void deleteFile() {
        if (fileIsBeingAccessed) {
            try {
                wait();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }
        fileIsBeingAccessed = true;
        if (writer != null) {
            try {
                writer.close();
            } catch (IOException e) {
                PrintLogger.print("Failed to close file writer: " + "\n" + e.getMessage());
            }
        }
        logFile.delete();
        writer = null;
        fileIsBeingAccessed = false;
        notifyAll();
    }

    public synchronized boolean writeToFile(String str) {
    
        if (fileIsBeingAccessed) {
            try {
                wait();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return false;
            }
        }
        if(writer == null){
            return false;
        }
        fileIsBeingAccessed = true;
        str = str + "\n";
        try {
            writer.write(str);
            writer.flush();
        } catch (IOException e) {
            PrintLogger.print("Exception thrown while attempting to write to log file: " + "\n" + e.getMessage());
        }
        fileIsBeingAccessed = false;
        notifyAll();
        return true;
    }

    public String getFilePath() {
        return logPath;
    }
}
