package org.usd232.robotics.rapidreact.log;

import java.io.IOException;
import java.io.PrintStream;
import java.io.UnsupportedEncodingException;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import javax.net.ServerSocketFactory;

/**
 * The log server for logging data on the robot.
 * 
 * @author Zach, Brian
 */
public class LogServer implements Runnable {
    /**
     * The logger.
     * 
     * @since 2018
     * 
     */
    private static final Logger LOG = new Logger();
    
    /**
     * The port number for the logger.
     * 
     * @since 2018
     * 
     */
    private static final int PORT_NUM = 5800;
    /**
     * The list of sockets.
     * 
     * @since 2018
     * 
     */
    private final List<Socket> socketList = new LinkedList<Socket>();
    /**
     * The list of log servers.
     * 
     * @since 2018
     * 
     */
    private static final List<LogServer> servers = new ArrayList<LogServer>();

    /**
     * Emits a byte[] to every log server.
     * 
     * @param buffer The byte[] to log to the log server.
     * @throws IOException
     */
    static void emit(byte[] buffer) throws IOException {
        for (LogServer server : servers) {
            server.write(buffer);
        }
    }

    /**
     * Serializes a byte[].
     * 
     * @param b      The byte[] to serialize.
     * @param off    offset to start at.
     * @param len    The length of the byte[] to read.
     * @param date   The date it was logged at (in milliseconds)
     * @param level  The level of what is being logged.
     * @param logger The class that is logging it.
     * @return the serialized version of the log statement.
     * @throws IOException
     */
    static byte[] serialize(byte[] b, int off, int len, long date, LogLevel level, String logger) throws IOException {
        byte[] loggerBytes = logger.getBytes("UTF-8");
        byte[] buffer = new byte[4 + len + 8 + 1 + 4 + loggerBytes.length];
        ByteBuffer buf = ByteBuffer.wrap(buffer);
        buf.putInt(len);
        buf.put(b, off, len);
        buf.putLong(date);
        buf.put((byte) level.getLevelId());
        buf.putInt(loggerBytes.length);
        buf.put(loggerBytes);
        return buffer;
    }

    /**
     * Writes the byte array as a serialized log.
     * 
     * @param buffer the log in serialized form.
     * @see LogServer#serialize(byte[], int, int, long, LogLevel, String)
     * @throws IOException
     */
    public void write(byte[] buffer) throws IOException {
        for (Socket socket : socketList) {
            socket.getOutputStream().write(buffer);
            socket.getOutputStream().flush();
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void run() {
        LOG.catchAll(() -> {
            try {
                System.setOut(new PrintStream(new Printer(System.out, this, LogLevel.STDOUT), false, "UTF-8"));
            } catch (UnsupportedEncodingException ex) {
                ex.printStackTrace();
                System.setOut(new PrintStream(new Printer(System.out, this, LogLevel.STDOUT), false));
            }
            try {
                System.setErr(new PrintStream(new Printer(System.err, this, LogLevel.STDERR), false, "UTF-8"));
            } catch (UnsupportedEncodingException ex) {
                ex.printStackTrace();
                System.setErr(new PrintStream(new Printer(System.err, this, LogLevel.STDERR), false));
            }
            ServerSocketFactory serverSocketFactory = ServerSocketFactory.getDefault();
            ServerSocket serverSocket = null;
            try {
                serverSocket = serverSocketFactory.createServerSocket(PORT_NUM);
            } catch (IOException ignored) {
                System.err.println("Unable to create server");
                System.exit(-1);
            }
            LOG.info("LogServer running on port %s", PORT_NUM);
            while(true) {
                Socket socket = null;
                try {
                    socket = serverSocket.accept();
                    socketList.add(socket);
                    LOG.info("Socket at %s has successfully connected.", socket.getInetAddress());
                } catch (IOException ex) {
                    ex.printStackTrace();
                }
            }
        });
    }

    /**
     * Creates a log server.
     * 
     * @since 2018
     * 
     */
    public LogServer() {
        servers.add(this);
    }

    /**
     * I dont think I need to document this.
     * 
     * @param args I don't know (very sarcastic).
     */
    public static void main(String[] args) {
        new LogServer().run();
    }
}