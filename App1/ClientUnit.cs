﻿using CanTest;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Windows.Networking;
using Windows.Networking.Sockets;
using Windows.Storage.Streams;

namespace App1
{
    class ClientUnit
    {
        private GlobalDataSet globalDataSet;

        private StreamSocket socket_client_receive, socket_client_send;
        private DataWriter dataWriter;
        private DataReader dataReader;

        public delegate void Error(string message);
        public event Error OnError;

        public delegate void DataRecived(string data);
        public event DataRecived OnDataRecived;

        public ClientUnit(GlobalDataSet globalDataSet)
        {
            this.globalDataSet = globalDataSet;
        }

        public async void StartClient()
        {
            try
            {
                var hostName = new HostName(globalDataSet.HostIp);

                // Connect to server send socket
                socket_client_receive = new StreamSocket();
                await socket_client_receive.ConnectAsync(hostName, globalDataSet.HostPortSend.ToString());
                dataReader = new DataReader(socket_client_receive.InputStream);

                // Connect to server receive socket
                socket_client_send = new StreamSocket();
                await socket_client_send.ConnectAsync(hostName, globalDataSet.HostPortReceive.ToString());
                dataWriter = new DataWriter(socket_client_send.OutputStream);
                clientServerLoop();
            }
            catch (Exception ex)
            {
                if (OnError != null)
                    OnError(ex.Message);
            }
        }

        public async void sendData(Byte[] message)
        {
            dataWriter.WriteBytes(message);
            //for (int i = 0; i < message.Length; i++) Debug.WriteLine("send message[" + i + "]" + message[i]);
            await dataWriter.StoreAsync();
            await dataWriter.FlushAsync();
            //try
            //{

            //}
            //catch (Exception ex)
            //{
            //    if (OnError != null)
            //        OnError(ex.Message);
            //}
        }

        // Check different task numbers and do some work
        private async void clientServerLoop()
        {
            byte[] receiveBytes = new byte[8];
            byte[] sendBytes = new byte[8];
            //try
            //{
            while (true)
            {
                // Read data from server
                uint sizeFieldCount = await dataReader.LoadAsync(8);
                dataReader.ReadBytes(receiveBytes);
                //for (int i = 0; i < receiveBytes.Length; i++) Debug.WriteLine("receiveBytes[" + i + "] " + receiveBytes[i]);
                Debug.WriteLine("receiveBytes[" + 0 + "] " + receiveBytes[0]);

                // Set incoming data to global data
                globalDataSet.Incoming_DataPackage = receiveBytes;

                // Get data from global data
                //for (int i = 0; i < sendBytes.Length; i++)
                //{
                    //sendBytes = globalDataSet.Outgoing_DataPackage;
                    //Debug.WriteLine("sendBytes[" + i + "]: " + sendBytes[i]);
                    //Debug.WriteLine("receiveBytes[" + i + "]: " + receiveBytes[i]);
                //}
                // Send data to server 
                sendData(globalDataSet.Outgoing_DataPackage);


                //uint sizeFieldCount = await dataReader.LoadAsync(8);
                //if desconneted
                //if (sizeFieldCount != sizeof(uint))
                //    return;

                //dataReader.ReadBytes(receiveBytes);
                //uint actualStringLength = await _reader.LoadAsync(stringLenght);
                ////if desconneted
                //if (stringLenght != actualStringLength)
                //    return;
                //if (OnDataRecived != null)
                //    OnDataRecived(dataReader.ReadString(actualStringLength));
            }

            //}
            //catch (Exception ex)
            //{
            //    if (OnError != null)
            //        OnError(ex.Message);
            //}
        }


        public void closeConnection()
        {
            dataWriter.DetachStream();
            dataWriter.Dispose();

            dataReader.DetachStream();
            dataReader.Dispose();

            socket_client_send.Dispose();
            socket_client_receive.Dispose();
        }
    }
}
