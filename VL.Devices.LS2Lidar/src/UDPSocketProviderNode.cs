using System.Net;
using System.Net.Sockets;
using VL.Core.Import;
using VL.Lib.Basics.Resources;
using NetSocket = System.Net.Sockets.Socket;

namespace VL.Devices.LS2Lidar
{
    [ProcessNode]
    public class UDPSocketProviderNode
    {
        ResourceProviderMonitor<NetSocket>? _provider;
        private bool _connect = false;
        private IPEndPoint? _remoteEndpoint;

        public IResourceProvider<NetSocket> Update(
            IPEndPoint remoteEndpoint,
            bool connect,
            bool enabled = true
        )
        {
            if (
                !UDPSocketProviderNode.IPEndPointEquals(_remoteEndpoint, remoteEndpoint)
                || _connect != connect
            )
            {
                _connect = connect;
                _remoteEndpoint = remoteEndpoint;

                _provider = ResourceProvider
                    .New(() =>
                    {
                        var socket = new NetSocket(SocketType.Dgram, ProtocolType.Udp);
                        socket.ExclusiveAddressUse = false;
                        socket.SetSocketOption(
                            SocketOptionLevel.Socket,
                            SocketOptionName.ReuseAddress,
                            true
                        );
                        if (connect && remoteEndpoint != null)
                            socket.Connect(remoteEndpoint);
                        return socket;
                    })
                    .ShareInParallel()
                    .Monitor();
            }

            if (enabled)
                return _provider;
            return null;
        }

        public bool IsOpen => _provider?.SinkCount > 0;

        public static bool IPEndPointEquals(IPEndPoint? input, IPEndPoint? input2)
        {
            if (ReferenceEquals(input, input2))
                return true;
            if (ReferenceEquals(input, null))
                return false;
            return input.Equals(input2);
        }
    }
}
