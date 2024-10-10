package frc.team4276.frc2024.Logging;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.channels.FileChannel;
import java.nio.channels.OverlappingFileLockException;
import java.nio.file.*;
import java.nio.file.attribute.BasicFileAttributes;
import java.time.LocalTime;
import java.time.format.DateTimeFormatter;
import java.util.Random;
import edu.wpi.first.wpilibj.Timer;
import frc.team4276.frc2024.Constants.DebugConstants;
//can still be used single threaded 
/*
 * example usage:
 *    //class will handle the file extensions for you do not add a file extension
 *    RobotFileLogger logger = new RobotFileLogger("test");
 *    logger.init();
 *    logger.writeToFile("testing", RobotFileLogger.DebugLevel.DEBUG);
 */
public class LoggableRobotFile implements Runnable {
    private final File logFile;
    private FileWriter writer;
    private final String logPath;
    private static String logDirectory = DebugConstants.logDirectory;
    private boolean fileIsBeingAccessed;
    private String outString;
    private DebugLevel outDebug;
    public boolean isNewData;
    @SuppressWarnings("unused")
    private boolean stopThread = false;

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
    }

    private static long sessionID = 1000000000L + (long) (new Random().nextDouble() * 9000000000L);

    public LoggableRobotFile(String fileName) {
        fileName = fileName + ".rlog";
        String modifiedFileName = addTimestampAndSessionID(fileName, String.valueOf(sessionID));
        logPath = logDirectory + modifiedFileName;
        logFile = new File(logPath);
    }

    @SuppressWarnings("unused")
    public void init() {
        //no dead code just because of a condition on a constant 
        assert DebugConstants.maxDirSize > DebugConstants.reductionSize : "maxDirSize must be greater than reduction size";
        assert DebugConstants.logDirectory != "" : "logDirectory cannot be blank";
        if(logDirectory.lastIndexOf("/") != logDirectory.length() - 1){
            PrintLogger.print("logDirectory must end with a slash, logging will not be avaliable");
            return;
        }
        try {
            checkAndReduceDirectorySize(Paths.get(logDirectory));
            logFile.createNewFile();
            writer = new FileWriter(logPath, true);
        } catch (IOException e) {
            PrintLogger.print("Exception thrown while creating file logger, file logging will not be available: " + "\n" + e.getStackTrace());
            writer = null;
        }
    }

    public synchronized void setDebugSetString(String str, DebugLevel level) {
        outString = str;
        outDebug = level;
        isNewData = true;
        notifyAll(); // Notify that new data is available
    }

    public synchronized void run() {
        while (true) {
            try {
                while (!isNewData) {
                    wait(); 
                }
                if (isNewData) {
                    writeToFile(outString, outDebug);
                    isNewData = false;
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }
    }

    public synchronized void clearFile() {
        fileIsBeingAccessed = true;
        if (writer != null) {
            try {
                writer.close();
                logFile.delete();
                logFile.createNewFile();
                writer = new FileWriter(logPath);
            } catch (IOException e) {
                PrintLogger.print("Failed to clear file: " + "\n" + e.getMessage());
            }
        }
        fileIsBeingAccessed = false;
    }

    public synchronized void deleteFile() {
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
    }

    public synchronized boolean writeToFile(String str, DebugLevel level) {
        if (writer == null) {
            return false;
        }
        fileIsBeingAccessed = true;
        str = str + "\n";
        try {
            writer.write("[Time:" + Timer.getFPGATimestamp() + " Session ID:" + sessionID + " Level:" + level.getLevelString() + "] ");
            writer.write(str);
            writer.flush();
        } catch (IOException e) {
            PrintLogger.print("Exception thrown while attempting to write to log file: " + "\n" + e.getMessage());
            return false;
        } finally {
            fileIsBeingAccessed = false;
            notifyAll(); // Notify that file access is complete
        }
        return true;
    }

    public String getFilePath() {
        return logPath;
    }

    public synchronized boolean isFileInUse() {
        return fileIsBeingAccessed;
    }

    private static void checkAndReduceDirectorySize(Path dir) throws IOException {
        if (getDirectorySizeInMB(dir) >= DebugConstants.maxDirSize) {
            while (getDirectorySizeInMB(dir) > DebugConstants.maxDirSize - DebugConstants.reductionSize && !areAllFilesNotRlog(dir)) {
                String oldestFilePath = findOldestRlogFile(dir);
                if (isFileInUse(Paths.get(oldestFilePath)) || !validRlog(oldestFilePath)) {
                    continue;
                }
                Files.delete(Paths.get(oldestFilePath));
            }
        }
    }

    private static double getDirectorySizeInMB(Path dir) throws IOException {
        final long[] totalSize = {0};

        Files.walkFileTree(dir, new SimpleFileVisitor<Path>() {
            @Override
            public FileVisitResult visitFile(Path file, BasicFileAttributes attrs) throws IOException {
                if (Files.isRegularFile(file)) {
                    totalSize[0] += Files.size(file);
                }
                return FileVisitResult.CONTINUE;
            }
        });

        return totalSize[0] / (1024.0 * 1024.0);
    }

    private static String findOldestRlogFile(Path dir) throws IOException {
        final Path[] oldestFile = {null};
        final long[] oldestTime = {Long.MAX_VALUE};

        Files.walkFileTree(dir, new SimpleFileVisitor<Path>() {
            @Override
            public FileVisitResult visitFile(Path file, BasicFileAttributes attrs) throws IOException {
                if (Files.isRegularFile(file) && validRlog(file.toString())) {
                    long fileTime;
                    try {
                        fileTime = attrs.creationTime().toMillis();
                    } catch (UnsupportedOperationException e) {
                        fileTime = attrs.lastModifiedTime().toMillis();
                    }

                    if (fileTime < oldestTime[0]) {
                        oldestTime[0] = fileTime;
                        oldestFile[0] = file;
                    }
                }
                return FileVisitResult.CONTINUE;
            }
        });

        return oldestFile[0] != null ? oldestFile[0].toString() : null;
    }

    private static String addTimestampAndSessionID(String fileName, String sessionID) {
        LocalTime currentTime = LocalTime.now();
        DateTimeFormatter formatter = DateTimeFormatter.ofPattern("HH-mm-ss");
        String formattedTime = currentTime.format(formatter);

        int dotIndex = fileName.lastIndexOf('.');
        String namePart = dotIndex == -1 ? fileName : fileName.substring(0, dotIndex);
        String extensionPart = dotIndex == -1 ? "" : fileName.substring(dotIndex);

        return namePart + "[" + formattedTime + "][" + sessionID + "]" + extensionPart;
    }

    private static boolean validRlog(String fileName) {
        return fileName.toLowerCase().endsWith(".rlog");
    }
    //os will deny multiple locks on a file 
    private static boolean isFileInUse(Path filePath) throws IOException {
        try (FileChannel fileChannel = FileChannel.open(filePath, StandardOpenOption.WRITE)) {
            fileChannel.tryLock();
            return false;
        } catch (OverlappingFileLockException e) {
            return true;
        }
    }

    private static boolean areAllFilesNotRlog(Path dir) throws IOException {
        final boolean[] noneRlog = {true};

        Files.walkFileTree(dir, new SimpleFileVisitor<Path>() {
            @Override
            public FileVisitResult visitFile(Path file, BasicFileAttributes attrs) throws IOException {
                if (Files.isRegularFile(file)) {
                    if (validRlog(file.toString())) {
                        noneRlog[0] = false;
                        return FileVisitResult.TERMINATE;
                    }
                }
                return FileVisitResult.CONTINUE;
            }

            @Override
            public FileVisitResult preVisitDirectory(Path dir, BasicFileAttributes attrs) throws IOException {
                return dir.equals(dir) ? FileVisitResult.CONTINUE : FileVisitResult.SKIP_SUBTREE;
            }
        });

        return noneRlog[0];
    }
}
