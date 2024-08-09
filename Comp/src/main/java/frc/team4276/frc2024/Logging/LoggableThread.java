package frc.team4276.frc2024.Logging;

import java.util.logging.Logger;

import frc.team4276.frc2024.Logging.LoggableRobotFile.DebugLevel;

public class LoggableThread {
    private LoggableRobotFile mLogger;
    private Thread mThread;
    public LoggableThread(String fileName){
        mLogger = new LoggableRobotFile(fileName);
    }
    public void init(){
        mLogger.init();
        mThread = new Thread(mLogger);
    }
    
    public void logAsync(String str, DebugLevel level){
        //wait for the file to not be in use 
        while(mLogger.isFileInUse()){}
        mLogger.setDebugSetString(str, level);
        if(isRunning() == false){
            log();
        }
        
    }
    public void deleteFile(){
        try {
            mThread.join();
            mLogger.deleteFile();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    public void clearFile(){
        while(mLogger.isFileInUse()){}
        mLogger.clearFile();
    }
    public String getPath(){
        return mLogger.getFilePath();
    }
    public void KillThread() {
        try {
            mThread.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    //returns true if the thread is still running false other wise
    public boolean isRunning(){
        return mThread.isAlive();
    }
    private void log(){
        mThread.start();
    }
}
