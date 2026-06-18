using System.Net;
using System.Net.Sockets;
using VL.Lib.Basics.Resources;
using NetSocket = System.Net.Sockets.Socket;

namespace VL.Devices.LS2Lidar
{
    public interface ILS2LidarSocketProvider : IDisposable
    {
        IResourceProvider<NetSocket>? SocketProvider { get; }

        bool IsOpen { get; }

        void Connect(IPEndPoint remoteEndpoint);

        void Disconnect();
    }

    public class LS2LidarSocketProvider : ILS2LidarSocketProvider
    {
        private ResourceProviderMonitor<NetSocket>? _provider;
        private IPEndPoint? _remoteEndpoint;

        public IResourceProvider<NetSocket>? SocketProvider => _provider;

        public bool IsOpen => _provider?.SinkCount > 0;

        public void Connect(IPEndPoint remoteEndpoint)
        {
            if (_provider != null && IPEndPointHelper.IPEndPointEquals(remoteEndpoint, _remoteEndpoint))
                return;

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
                    socket.Connect(remoteEndpoint);
                    return socket;
                })
                .ShareInParallel()
                .Monitor();
        }

        public void Disconnect()
        {
            _provider = null;
            _remoteEndpoint = null;
        }

        public void Dispose()
        {
            Disconnect();
        }
    }

    public static class IPEndPointHelper
    {
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
