package frc.team4276.frc2024.Logging;

import frc.team4276.frc2024.Logging.LoggableRobotFile.DebugLevel;
/*
 * example usage:
 *  LoggableThread logThread = new LoggableThread("ThreadTest");
 *  logThread.init();
 *  logThread.logAsync("fizz", LoggableRobotFile.DebugLevel.INFO);
 *  //buzz will wait for fizz to be printed 
 *  logThread.logAsync("buzz", LoggableRobotFile.DebugLevel.INFO);
 */
public class LoggableThread {
    private LoggableRobotFile mLogger;
    private Thread mThread;

    public LoggableThread(String fileName) {
        mLogger = new LoggableRobotFile(fileName);
    }

    public void init() {
        mLogger.init();
        mThread = new Thread(mLogger);
        mThread.start(); 
    }

    //each call will wait for the previous to finish 
    public void logAsync(String str, DebugLevel level) {
        while (mLogger.isFileInUse() || mLogger.isNewData) {}
        mLogger.setDebugSetString(str, level);
    }

    public void deleteFile() {
        stopLogging();
        try {
            mThread.join();
            mLogger.deleteFile();
            killThread();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void clearFile() {
        while (mLogger.isFileInUse()) {}
        mLogger.clearFile();
    }

    public String getPath() {
        return mLogger.getFilePath();
    }

    public void killThread() {
        stopLogging();
        try {
            mThread.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private void stopLogging() {
        if (mThread != null) {
            mThread.interrupt();
        }
    }

    public boolean isRunning() {
        return mThread != null && mThread.isAlive();
    }
}