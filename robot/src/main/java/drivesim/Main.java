package drivesim;

import com.coolioasjulio.rpc.server.RPCServer;

import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;

public class Main
{
    public static void main(String[] args)
    {
        try (ServerSocket serverSocket = new ServerSocket(4444))
        {
            while (true)
            {
                System.out.println("Waiting for connection...");
                Socket socket = serverSocket.accept();
                System.out.println("Received connection from " + socket.getInetAddress().toString());

                RPCServer.getInstance().setLoggingEnabled(false);
                RPCServer.RPCSession session = RPCServer.getInstance()
                    .createRPCSession(socket.getInputStream(), socket.getOutputStream());
                try
                {
                    session.join();
                }
                catch (InterruptedException e)
                {
                    e.printStackTrace();
                }
            }
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
    }
}
