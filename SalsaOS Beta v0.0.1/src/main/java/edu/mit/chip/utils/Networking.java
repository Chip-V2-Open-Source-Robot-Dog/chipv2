package edu.mit.chip.utils;

import java.util.HashMap;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Networking {
    private static Networking instance;
    private HashMap<String, NetworkTableEntry> entries;
    private final NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
    private final NetworkTable table = networkTableInstance.getTable("chip");
    
    private Networking(String... entries) {
        this.entries = new HashMap<String, NetworkTableEntry>();

        for (String entry : entries) {
            this.entries.put(entry, table.getEntry(entry));
        }
    }

    public static void init(String... entries) {
        instance = new Networking(entries);
    }

    public static Networking getInstance(String... entries) {
        if (instance == null) {
            init(entries);
        }
        return instance;
    }

    public void pushDouble(String entry, double value) {
        entries.get(entry).setDouble(value);
    }
}