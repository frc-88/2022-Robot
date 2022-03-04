package frc.robot.util.tunnel;

import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
import java.util.Objects;

public class TunnelServerThreadless {

    private int port = 0;
    private TunnelInterface tunnel_interface;
    private ArrayList<TunnelClient> clients;

    public static TunnelServerThreadless instance = null;
    ServerSocket m_serverSocket = null;
    Socket m_socket = null;
    private boolean is_started = false;

    public TunnelServerThreadless(TunnelInterface tunnel_interface, int port)
    {
        if (!Objects.isNull(instance)) {
            throw new RuntimeException("Only once instance of TunnelServerThreadless allowed");
        }
        instance = this;

        clients = new ArrayList<TunnelClient>();
        this.port = port;
        this.tunnel_interface = tunnel_interface;
    }

    public static boolean anyClientsAlive()
    {
        for (int index = 0; index < TunnelServerThreadless.instance.clients.size(); index++)
        {
            TunnelClient client = TunnelServerThreadless.instance.clients.get(index);
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
        for (int index = 0; index < TunnelServerThreadless.instance.clients.size(); index++)
        {
            TunnelClient client = TunnelServerThreadless.instance.clients.get(index);
            if (client.isAlive() && client.isOpen()) {
                client.writePacket(category, objects);
            }
        }
    }

    // Send message to all clients
    public static void println(String message) {
        TunnelServerThreadless.writePacket("__msg__", message);
    }

    private void start() {
        
        try {
            m_serverSocket = new ServerSocket(port);
        } catch (IOException e) {
            e.printStackTrace();
            return;
        }
        System.out.println("Socket is open");
    }

    public void update() {
        if (!is_started) {
            start();
            is_started = true;
        }
        try
        {
            try {
                m_socket = m_serverSocket.accept();
            } catch (IOException e) {
                System.out.println("I/O error: " + e);
            }
            cleanUpThreads();

            // new threads for a client
            TunnelClient client = new TunnelClient(tunnel_interface, m_socket);
            clients.add(client);
            client.start();
        }
        finally {
            try {
                System.out.println("Closing socket");
                m_serverSocket.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
            is_started = false;
        }
    }
}
