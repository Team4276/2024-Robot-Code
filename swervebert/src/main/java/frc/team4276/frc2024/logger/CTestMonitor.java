//*****************************************************************************************//
// The MIT License (MIT)                                                                   //
//                                                                                         //
// Copyright (c) 2017 - Marina High School FIRST Robotics Team 4276 (Huntington Beach, CA) //
//                                                                                         //
// Permission is hereby granted, free of charge, to any person obtaining a copy            //
// of this software and associated documentation files (the "Software"), to deal           //
// in the Software without restriction, including without limitation the rights            //
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell               //
// copies of the Software, and to permit persons to whom the Software is                   //
// furnished to do so, subject to the following conditions:                                //
//                                                                                         //
// The above copyright notice and this permission notice shall be included in              //
// all copies or substantial portions of the Software.                                     //
//                                                                                         //
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR              //
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,                //
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE             //
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER                  //
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,           //
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN               //
// THE SOFTWARE.                                                                           //
//*****************************************************************************************//
//*****************************************************************************************//
// We are a high school robotics team and always in need of financial support.             //
// If you use this software for commercial purposes please return the favor and donate     //
// (tax free) to "Marina High School Educational Foundation, attn: FRC team 4276"          //
// (Huntington Beach, CA)                                                                  //
//*****************************************************************************************//
package frc.team4276.frc2024.logger;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Calendar;
import java.util.Date;
import java.util.logging.Level;
import java.util.logging.Logger;

public final class CTestMonitor {

    Boolean m_isPrintEnabled = false;
    Boolean m_isMonitorEnabled = false;
    int m_nMaxNumberOfFiles = 10;
    int m_nMaxFileSize = 5 * 1024 * 1024; // 5 MegaBytes;

    final String HOME_NAME = "admin";

    String m_sBaseFileName;
    String m_sLogFolder;

    String m_path;
    File m_file;
    FileWriter m_fileWriter;
    BufferedWriter m_output;

    public CTestMonitor() {
        init();
    }

    public void init() {

        Date date = new Date();
        Calendar cal = Calendar.getInstance();
        cal.setTime(date);

        m_sBaseFileName = numberToText00(cal.get(Calendar.YEAR) + 1900);
        m_sBaseFileName += numberToText00(cal.get(Calendar.MONTH) + 1);
        m_sBaseFileName += numberToText00(cal.get(Calendar.DAY_OF_MONTH) + 1);
        m_sBaseFileName += "-";
        m_sBaseFileName += numberToText00(cal.get(Calendar.HOUR_OF_DAY));
        m_sBaseFileName += "-";
        m_sBaseFileName += numberToText00(cal.get(Calendar.MINUTE));
        m_sBaseFileName += "-";
        m_sBaseFileName += numberToText00(cal.get(Calendar.SECOND));

        m_sLogFolder = "/home/";
        m_sLogFolder += HOME_NAME;
        m_sLogFolder += "/log";

        m_path = getLogFilePath();

        limitMaxFiles();

        try {

            m_file = new File(m_path);

            // Creates a FileWriter
            m_fileWriter = new FileWriter(m_path);

            // Creates a BufferedWriter
            m_output = new BufferedWriter(m_fileWriter);
        } catch (IOException ex) {
            Logger.getLogger(CTestMonitor.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    private void limitMaxFiles() {
        File dir = new File(m_sLogFolder);
        int nFiles = dir.listFiles().length;
        if (nFiles > m_nMaxNumberOfFiles) {
            long oldestTime = 0;
            String sOldestFileName = "";
            File[] directoryListing = dir.listFiles();
            if (directoryListing != null) {
                for (File child : directoryListing) {
                    if (oldestTime == 0) {
                        oldestTime = child.lastModified();
                        sOldestFileName = child.getName();
                    } else {
                        if (oldestTime > child.lastModified()) {
                            oldestTime = child.lastModified();
                            sOldestFileName = child.getName();
                        }
                    }
                }
                File deleteThisFile = new File(m_sLogFolder + "/" + sOldestFileName);
                deleteThisFile.delete();
            }
        }
    }

    public boolean isTestMonitorEnabled() {
        return m_isMonitorEnabled;
    }

    public String numberToText(int n) {
        Integer i = n;
        return i.toString();
    }

    public String numberToText00(int n) {
        return String.format("%02d", n);
    }

    public String numberToText0000(int n) {
        return String.format("%04d", n);
    }

    public int atoi(String str) {
        if (str == null || str.length() < 1) {
            return 0;
        }

        // trim white spaces
        str = str.trim();

        char flag = '+';

        // check negative or positive
        int i = 0;
        if (str.charAt(0) == '-') {
            flag = '-';
            i++;
        } else if (str.charAt(0) == '+') {
            i++;
        }
        // use double to store result
        double result = 0;

        // calculate value
        while (str.length() > i && str.charAt(i) >= '0' && str.charAt(i) <= '9') {
            result = result * 10 + (str.charAt(i) - '0');
            i++;
        }

        if (flag == '-') {
            result = -result;
        }

        // handle max and min
        if (result > Integer.MAX_VALUE) {
            return Integer.MAX_VALUE;
        }

        if (result < Integer.MIN_VALUE) {
            return Integer.MIN_VALUE;
        }

        return (int) result;
    }

    public String getLogFilePath() {
        String sRet = m_sLogFolder;
        sRet += "/log-";
        sRet += m_sBaseFileName;
        sRet += ".txt";
        return sRet;
    }

    public void deleteFileByNumberIfExists(int nFile, String sFolderPath) {
        String sFile = sFolderPath;
        sFile += "/";
        sFile += numberToText0000(nFile);
        sFile += "*.*";
        File f = new File(sFile);
        f.delete();
    }

    public void dbgMsg_s(String str) {
        System.out.print(str);
    }

    public Boolean logWrite(String sLine) {

        if (m_isPrintEnabled) {
            dbgMsg_s(sLine);
        }
        if (m_isMonitorEnabled) {
            try {
                if (m_file.exists()) {
                    if (m_file.length() > m_nMaxFileSize) {
                        m_output.close();
                        init();
                    }
                }
                m_output.append(sLine);
                m_output.flush();
            } catch (IOException ex) {
                Logger.getLogger(CTestMonitor.class.getName()).log(Level.SEVERE, null, ex);
            }
        }
        return true;
    }

    public long getTicks() {
        return System.currentTimeMillis();
    }

    public long getDeltaTimeSeconds(long timeStart, long timeEnd) {
        long dTemp = getDeltaTimeMilliseconds(timeStart, timeEnd);
        dTemp /= 1000.0;
        return dTemp;
    }

    public long getDeltaTimeMilliseconds(long timeStart, long timeEnd) {
        return timeEnd - timeStart;
    }

    public void padString(String str, int desiredLength) {
        while (str.length() < desiredLength) {
            str += " ";
        }
    }
}
