package frc.lib.util;

import java.util.List;

public class DataProcessor implements Runnable {
    public interface ReaderLoggerData {
        void readLogDataFromIO();
    }

    public interface IODataRefresher {
        void refreshData();
    }

    public static final int LOOP_TIME = 20;

    public static void initDataProcessor(
        ReaderLoggerData readerLogger, IODataRefresher dataRefresher) {
            new Thread(new DataProcessor(readerLogger, dataRefresher)).start();
    }

    private double timestamp = 0.0;
    private ReaderLoggerData dataReaderLogger;
    private List<IODataRefresher> IODataRefreshers;

    public DataProcessor(
        ReaderLoggerData readerLogger,
        IODataRefresher dataRefresher
    ) {
        this.dataReaderLogger = readerLogger;
        IODataRefreshers = List.of(dataRefresher);
    }

    public DataProcessor(
        ReaderLoggerData readerLogger,
        IODataRefresher dataRefresher1,
        IODataRefresher dataRefresher2,
        IODataRefresher dataRefresher3
    ) {
        this.dataReaderLogger = readerLogger;
        IODataRefreshers = List.of(dataRefresher1, dataRefresher2, dataRefresher3);
    }

    public void run() {
        while (true) {
            timestamp = System.currentTimeMillis();
            for(IODataRefresher i : IODataRefreshers) {
                i.refreshData();
            }

            dataReaderLogger.readLogDataFromIO();
            try {
                var delta = System.currentTimeMillis() - timestamp;
                if(delta < LOOP_TIME) {
                    Thread.sleep((long)(LOOP_TIME-delta));
                }
            } catch(InterruptedException e) {}
        }
    }
    
}
