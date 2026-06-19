using System.Net;
using System.Net.Sockets;
using System.Reactive.Subjects;
using Devices.LS2Lidar;
using Devices.LS2Lidar.Model;
using VL.Core.Import;
using NetSocket = System.Net.Sockets.Socket;

namespace VL.Devices.LS2Lidar
{
    public enum LS2LidarState
    {
        Disconnected,
        Connected,
        Scanning,
        Error,
    }

    [ProcessNode(
        Name = "LS2Lidar",
        HasStateOutput = true,
        FragmentSelection = FragmentSelection.Explicit
    )]
    public class LS2LidarNode : IDisposable
    {
        // Core Logic Components (Zero-allocation processors)
        private readonly FrameAssembler _assembler = new FrameAssembler();
        private readonly LidarParser _parser = new LidarParser();

        // Pre-allocated container to completely eliminate GC allocations
        private readonly ScanData _reusableScanData = new ScanData();

        // vvvv Reactive Outputs
        private readonly Subject<ScanData> _scans = new Subject<ScanData>();

        // Networking State (Direct Socket Ownership)
        private NetSocket? _socket;
        private IPEndPoint? _remoteEndpoint;
        private CancellationTokenSource? _cancellation;
        private Task? _receiveTask;

        [Fragment]
        public LS2LidarState LidarState { get; private set; } = LS2LidarState.Disconnected;

        [Fragment]
        public IObservable<ScanData> Scans => _scans;

        [Fragment]
        public LS2LidarNode() { }

        public void Connect(string ipAddress, int port = 2112)
        {
            if (!IPAddress.TryParse(ipAddress, out IPAddress? parsedAddress))
            {
                LidarState = LS2LidarState.Error;
                return;
            }

            if (_receiveTask != null && !parsedAddress.Equals(_remoteEndpoint?.Address))
            {
                Disconnect();
            }

            if (_receiveTask == null)
            {
                try
                {
                    _remoteEndpoint = new IPEndPoint(parsedAddress, port);

                    _socket = new NetSocket(SocketType.Dgram, ProtocolType.Udp);
                    _socket.ExclusiveAddressUse = false;

                    // Bind to an OS-assigned free port (the device replies to whichever port we
                    // send from). Each instance gets its own port, so multiple lidars don't collide.
                    _socket.Bind(new IPEndPoint(IPAddress.Any, 0));

                    LidarState = LS2LidarState.Connected;
                    _cancellation = new CancellationTokenSource();
                    _receiveTask = Task.Run(() => ReceiveLoop(_socket, _cancellation.Token));
                }
                catch (Exception)
                {
                    LidarState = LS2LidarState.Error;
                    _socket?.Dispose();
                    _socket = null;
                }
            }
        }

        public void StartScanning()
        {
            SendCommand(Protocol.CMD_START_STREAM_DATA);
        }

        public void StopScanning()
        {
            SendCommand(Protocol.CMD_STOP_STREAM_DATA);
            if (LidarState == LS2LidarState.Scanning)
            {
                LidarState = LS2LidarState.Connected;
            }
        }

        private void SendCommand(ReadOnlySpan<byte> command)
        {
            if (_socket != null && _remoteEndpoint != null)
            {
                try
                {
                    _socket.SendTo(command.ToArray(), _remoteEndpoint);
                }
                catch (SocketException)
                {
                    LidarState = LS2LidarState.Error;
                }
            }
        }

        private async Task ReceiveLoop(NetSocket socket, CancellationToken ct)
        {
            byte[] receiveBuffer = new byte[Protocol.RECV_BUFFER_SIZE];

            try
            {
                while (!ct.IsCancellationRequested)
                {
                    int received = await socket.ReceiveAsync(receiveBuffer, SocketFlags.None, ct);

                    if (received <= 0)
                        continue;

                    ProcessDatagram(receiveBuffer, received);
                }
            }
            catch (OperationCanceledException) { }
            catch (ObjectDisposedException) { }
            catch (Exception)
            {
                LidarState = LS2LidarState.Error;
            }
            finally
            {
                if (LidarState == LS2LidarState.Scanning)
                {
                    LidarState = LS2LidarState.Connected;
                }
            }
        }

        private void ProcessDatagram(byte[] buffer, int received)
        {
            if (
                _assembler.TryAssemble(
                    buffer.AsSpan(0, received),
                    out ReadOnlySpan<byte> fullPayload
                )
            )
            {
                _parser.Parse(fullPayload, _reusableScanData);

                if (LidarState != LS2LidarState.Scanning)
                {
                    LidarState = LS2LidarState.Scanning;
                }

                _scans.OnNext(_reusableScanData);
            }
        }

        public void Disconnect()
        {
            _cancellation?.Cancel();
            _receiveTask?.Wait(500);
            _cancellation?.Dispose();
            _cancellation = null;
            _receiveTask = null;

            // Close and dispose the node's owned socket
            _socket?.Close();
            _socket?.Dispose();
            _socket = null;

            LidarState = LS2LidarState.Disconnected;
        }

        public void Dispose()
        {
            Disconnect();
            _scans.OnCompleted();
            _scans.Dispose();
        }
    }
}
