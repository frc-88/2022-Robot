package frc.robot.util.tunnel;

import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
import java.util.Objects;

public class TunnelServer extends Thread {

    private int port = 0;
    private TunnelInterface tunnel_interface;
    private ArrayList<TunnelClient> clients;
    private TunnelDataRelayThread data_relay_thread;

    public static TunnelServer instance = null;

    public TunnelServer(TunnelInterface tunnel_interface, int port, int data_relay_delay_ms, boolean auto_start)
    {
        if (!Objects.isNull(instance)) {
            throw new RuntimeException("Only once instance of TunnelServer allowed");
        }
        instance = this;

        clients = new ArrayList<TunnelClient>();
        this.port = port;
        this.tunnel_interface = tunnel_interface;
        this.data_relay_thread = new TunnelDataRelayThread(tunnel_interface, data_relay_delay_ms);

        if (auto_start) {
            this.start();
        }
    }

    public TunnelServer(TunnelInterface tunnel_interface, int port, int data_relay_delay_ms) {
        this(tunnel_interface, port, data_relay_delay_ms, true);
    }

    public static boolean anyClientsAlive()
    {
        for (int index = 0; index < TunnelServer.instance.clients.size(); index++)
        {
            TunnelClient client = TunnelServer.instance.clients.get(index);
            if (client.isAlive() && client.isOpen()) {
                return true;
            }
        }
        return false;
    }

    private void cleanUpThreads()
    {
        for (int index = 0; index < clients.size(); index++)
        {
            if (!clients.get(index).isAlive() || !clients.get(index).isOpen()) {
                clients.get(index).close();
                clients.remove(index);
                index--;
            }
        }
    }

    // Write a packet to all connected clients
    public static void writePacket(String category, Object... objects)
    {
        for (int index = 0; index < TunnelServer.instance.clients.size(); index++)
        {
            TunnelClient client = TunnelServer.instance.clients.get(index);
            if (client.isAlive() && client.isOpen()) {
                client.writePacket(category, objects);
            }
        }
    }

    // Send message to all clients
    public static void println(String message) {
        TunnelServer.writePacket("__msg__", message);
    }

    private void loop() {
        ServerSocket serverSocket = null;
        Socket socket = null;

        try {
            serverSocket = new ServerSocket(port);
        } catch (IOException e) {
            e.printStackTrace();
            return;
        }
        System.out.println("Socket is open");
        try
        {
            this.data_relay_thread.start();

            while (true) {
                try {
                    socket = serverSocket.accept();
                } catch (IOException e) {
                    System.out.println("I/O error: " + e);
                }
                cleanUpThreads();

                // new threads for a client
                TunnelClient client = new TunnelClient(tunnel_interface, socket);
                clients.add(client);
                client.start();
            }
        }
        finally {
            try {
                System.out.println("Closing socket");
                serverSocket.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
            try {
                Thread.sleep(500);
            }
            catch (InterruptedException e) {  }
        }
    }

    @Override
    public void run()
    {
        while (true) {
            loop();
        }
    }
}
