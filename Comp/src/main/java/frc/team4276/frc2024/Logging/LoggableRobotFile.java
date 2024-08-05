package frc.team4276.frc2024.Logging;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Random;

import edu.wpi.first.wpilibj.Timer;
import frc.team4276.frc2024.Constants.DebugConstants;

public class LoggableRobotFile {
/*
 * example usage:
 *    RobotFileLogger logger = new RobotFileLogger("test.log", true);
 *    logger.writeToFile("test", RobotFileLogger.DebugLevel.DEBUG);
 */
    private final File logFile;
    private FileWriter writer;
    private final String logPath;
    private static String logDirectory = DebugConstants.logDirectory;
    private boolean fileIsBeingAccessed;

    public enum DebugLevel {
        ERROR("error"),
        WARNING("warning"),
        INFO("info"),
        DEBUG("debug");
    
        private final String levelString;
    
        DebugLevel(String levelString) {
            this.levelString = levelString;
        }
    
        public String getLevelString() {
            return levelString;
        }
    }    //ten digit unique session id
    private static long sessionID = 1000000000L + (long)(new Random().nextDouble() * 9000000000L);
    //if clearFile is true this will clear the file before it begins writing 
    public LoggableRobotFile(String fileName, boolean clearFile) {
        logPath = logDirectory + fileName;
        logFile = new File(logPath);
        try {
            logFile.createNewFile();
            writer = new FileWriter(logPath, true);
        } catch (IOException e) {
            PrintLogger.print("Exception thrown while creating file logger, file logging will not be available: " + "\n" + e.getMessage());
            writer = null;
        }
        if(clearFile){
           clearFile();
        }
    }

    public synchronized void clearFile() {
        while (fileIsBeingAccessed) {
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
        while (fileIsBeingAccessed) {
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

    public synchronized boolean writeToFile(String str, DebugLevel level ) {
    
        while (fileIsBeingAccessed) {
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
            writer.write("[Time:" + "" +Timer.getFPGATimestamp() + " Session ID:" + sessionID + " Level:"+ level.levelString + "] ");
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
