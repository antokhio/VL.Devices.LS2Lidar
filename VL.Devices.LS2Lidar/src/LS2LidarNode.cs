using System.Net;
using System.Net.Sockets;
using System.Reactive.Subjects;
using Devices.LS2Lidar;
using VL.Core.Import;
using VL.Lib.Basics.Resources;
using NetSocket = System.Net.Sockets.Socket;

namespace VL.Devices.LS2Lidar
{
    public enum LS2LidarState
    {
        Idle,
        Scanning,
    }

    [ProcessNode(Name = "LS2Lidar", HasStateOutput = true)]
    public class LS2LidarNode : IDisposable
    {
        private const int DefaultPort = 2112;
        private const int ReceiveTimeoutMs = 1000;
        private const int ReceiveBufferSize = 65536;

        private readonly ILS2LidarSocketProvider _socketProvider = new LS2LidarSocketProvider();
        private readonly LS2LidarParser _parser = new();
        private readonly LS2LidarFrameAssembler _assembler = new();
        private readonly Subject<LS2LidarScanData> _scans = new();

        private IResourceHandle<NetSocket>? _socketHandle;
        private NetSocket? _socket;
        private IPEndPoint? _remoteEndpoint;

        private CancellationTokenSource? _cancellation;
        private Task? _receiveTask;

        public LS2LidarState LidarState { get; protected set; } = LS2LidarState.Idle;

        public bool IsConnected { get; private set; }

        public bool IsScanning { get; private set; }

        public IObservable<LS2LidarScanData> ScanData => _scans;

        public void Connect(string ipAddress, int port = DefaultPort)
        {
            var remoteEndpoint = new IPEndPoint(ResolveAddress(ipAddress), port);

            if (IsConnected && IPEndPointHelper.IPEndPointEquals(remoteEndpoint, _remoteEndpoint))
                return;

            Disconnect();

            _socketProvider.Connect(remoteEndpoint);
            _socketHandle = _socketProvider.SocketProvider!.GetHandle();
            _socket = _socketHandle.Resource;
            _remoteEndpoint = remoteEndpoint;
            IsConnected = true;
        }

        public void Disconnect()
        {
            StopScanning();

            _socketHandle?.Dispose();
            _socketHandle = null;
            _socket = null;
            _remoteEndpoint = null;

            _socketProvider.Disconnect();
            IsConnected = false;
        }

        public void StartScanning()
        {
            if (!IsConnected || _socket is null)
                throw new InvalidOperationException(
                    "Connect to the device before starting a scan."
                );

            if (IsScanning)
                return;

            _socket.ReceiveTimeout = ReceiveTimeoutMs;
            SendCommand(LS2LidarCommands.StartStreamData);

            _cancellation = new CancellationTokenSource();
            var socket = _socket;
            var token = _cancellation.Token;
            _receiveTask = Task.Run(() => ReceiveLoop(socket, token));

            IsScanning = true;
            LidarState = LS2LidarState.Scanning;
        }

        public void StopScanning()
        {
            if (!IsScanning)
                return;

            try
            {
                SendCommand(LS2LidarCommands.StopStreamData);
            }
            catch (SocketException)
            {
                // Ignore send failures while shutting the scan down.
            }

            _cancellation?.Cancel();
            try
            {
                _receiveTask?.Wait(TimeSpan.FromSeconds(2));
            }
            catch (AggregateException)
            {
                // The receive loop was cancelled.
            }

            _cancellation?.Dispose();
            _cancellation = null;
            _receiveTask = null;

            IsScanning = false;
            LidarState = LS2LidarState.Idle;
        }

        public void Reboot()
        {
            if (_socket is null)
                throw new InvalidOperationException("Connect to the device before rebooting.");

            StopScanning();
            SendCommand(LS2LidarCommands.Reboot);
        }

        private void SendCommand(ReadOnlySpan<byte> command)
        {
            _socket?.Send(command);
        }

        private void ReceiveLoop(NetSocket socket, CancellationToken token)
        {
            var buffer = new byte[ReceiveBufferSize];

            while (!token.IsCancellationRequested)
            {
                int received;
                try
                {
                    received = socket.Receive(buffer);
                }
                catch (SocketException ex) when (ex.SocketErrorCode == SocketError.TimedOut)
                {
                    continue;
                }
                catch (SocketException)
                {
                    break;
                }
                catch (ObjectDisposedException)
                {
                    break;
                }

                if (received <= 0)
                    continue;

                if (!_assembler.TryAssemble(buffer.AsSpan(0, received), out var payload))
                    continue;

                try
                {
                    var frame = new SensorFrame(payload);
                    _scans.OnNext(_parser.Parse(frame));
                }
                catch (Exception)
                {
                    // Skip malformed or incomplete frames.
                }
            }
        }

        private static IPAddress ResolveAddress(string host)
        {
            if (IPAddress.TryParse(host, out var address))
                return address;

            var addresses = Dns.GetHostAddresses(host);
            foreach (var candidate in addresses)
            {
                if (candidate.AddressFamily == AddressFamily.InterNetwork)
                    return candidate;
            }

            if (addresses.Length > 0)
                return addresses[0];

            throw new ArgumentException($"Could not resolve host '{host}'.", nameof(host));
        }

        public void Dispose()
        {
            Disconnect();

            _scans.OnCompleted();
            _scans.Dispose();
            _socketProvider.Dispose();
        }
    }
}
