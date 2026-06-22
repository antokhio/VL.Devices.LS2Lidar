using System.Buffers;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Reactive;
using System.Reactive.Subjects;
using Devices.LS2Lidar;
using Devices.LS2Lidar.Model;
using NetSocket = System.Net.Sockets.Socket;

namespace VL.Devices.LS2Lidar
{
    /// <summary>
    /// Abstraction over a single LS2 LiDAR device: owns a UDP socket, runs a background
    /// receive loop, assembles and parses datagrams into <see cref="Scan"/> values, and
    /// exposes connection state plus reactive notification streams.
    /// </summary>
    /// <remarks>
    /// Implementations are intended to be wrapped by a VL process node. Recoverable failures
    /// are surfaced through <see cref="OnError"/> rather than thrown, so the device can keep
    /// running after a transient error.
    /// </remarks>
    public interface ILS2Lidar : IDisposable
    {
        /// <summary>
        /// The remote device endpoint the socket is currently bound to, or <see langword="null"/>
        /// when not connected. Set on a successful <see cref="Connect(IPEndPoint)"/> and cleared on
        /// <see cref="Disconnect"/>.
        /// </summary>
        IPEndPoint? RemoteEndpoint { get; }

        /// <summary>
        /// <see langword="true"/> while a socket is open and connected to <see cref="RemoteEndpoint"/>;
        /// otherwise <see langword="false"/>. Reflects the node's own connection state rather than the
        /// raw UDP socket flag.
        /// </summary>
        bool IsConnected { get; }

        /// <summary>
        /// <see langword="true"/> while the background receive loop is running (i.e. the device is being
        /// polled for scan data); otherwise <see langword="false"/>.
        /// </summary>
        bool IsScanning { get; }

        /// <summary>
        /// Emits a snapshot <see cref="Scan"/> each time a complete datagram is assembled and parsed.
        /// Values are produced on the background receive thread.
        /// </summary>
        IObservable<Scan> Data { get; }

        /// <summary>
        /// Emits once each time a connection is successfully established via <see cref="Connect(IPEndPoint)"/>.
        /// </summary>
        IObservable<Unit> OnConnected { get; }

        /// <summary>
        /// Emits once each time an active connection is torn down via <see cref="Disconnect"/>.
        /// Does not emit when <see cref="Disconnect"/> is called while already disconnected.
        /// </summary>
        IObservable<Unit> OnDisconnected { get; }

        /// <summary>
        /// Emits the <see cref="Exception"/> for each recoverable error (connect failure, send failure,
        /// or a fault in the receive loop). The stream is not terminated by an error, so the device may
        /// continue operating afterwards.
        /// </summary>
        IObservable<Exception> OnError { get; }

        /// <summary>
        /// Parses <paramref name="remoteHost"/> and <paramref name="remotePort"/> into an endpoint and
        /// connects to it. An invalid host is reported through <see cref="OnError"/> rather than thrown.
        /// </summary>
        /// <param name="remoteHost">The device IP address, e.g. <c>"192.168.0.10"</c>.</param>
        /// <param name="remotePort">The device UDP port, e.g. <c>2112</c>.</param>
        void Connect(string remoteHost, int remotePort);

        /// <summary>
        /// Opens a UDP socket bound to an OS-assigned local port, connects it to
        /// <paramref name="remoteEndpoint"/>, and starts the background receive loop. If a connection is
        /// already active it is disconnected first. Connection failures are reported through
        /// <see cref="OnError"/>.
        /// </summary>
        /// <param name="remoteEndpoint">The remote device endpoint to connect to.</param>
        /// <exception cref="ArgumentNullException">
        /// Thrown when <paramref name="remoteEndpoint"/> is <see langword="null"/>.
        /// </exception>
        void Connect(IPEndPoint remoteEndpoint);

        /// <summary>
        /// Sends the "start streaming data" command to the connected device, instructing it to begin
        /// emitting scan datagrams.
        /// </summary>
        void StartScan();

        /// <summary>
        /// Sends a raw protocol command to the connected device. Does nothing (and reports through
        /// <see cref="OnError"/>) when not connected.
        /// </summary>
        /// <param name="command">The protocol bytes to transmit.</param>
        void SendCommand(ReadOnlySpan<byte> command);

        /// <summary>
        /// Cancels the receive loop, closes and disposes the socket, clears the endpoint, and emits
        /// <see cref="OnDisconnected"/> if a connection was active. Safe to call when already disconnected.
        /// </summary>
        void Disconnect();
    }

    public class LS2Lidar : ILS2Lidar
    {
        public const string DefaultRemoteHost = "192.168.0.10";
        public const int DefaultRemotePort = 2112;

        // Core Logic Components
        private readonly FrameAssembler _assembler = new FrameAssembler();
        private readonly LidarParser _parser = new LidarParser();

        // Pre-allocated container
        private readonly ScanData _scanData = new ScanData();

        // Networking State
        private NetSocket? _socket;
        private CancellationTokenSource? _cancellation;
        private Task? _receiveTask;
        private volatile bool _isConnected;

        // Reactive Subjects
        private readonly Subject<Scan> _scan = new();
        private readonly Subject<Unit> _onConnected = new();
        private readonly Subject<Unit> _onDisconnected = new();
        private readonly Subject<Exception> _onError = new();

        // Alive state
        private long _lastPacketTimestamp; // 0 = nothing received yet
        private readonly long _timeoutTicks = Stopwatch.Frequency / 2; // 500 ms silence => not live

        public IPEndPoint? RemoteEndpoint { get; private set; }

        public bool IsConnected => _isConnected;
        public bool IsScanning
        {
            get
            {
                long last = Interlocked.Read(ref _lastPacketTimestamp);
                return last != 0 && (Stopwatch.GetTimestamp() - last) <= _timeoutTicks;
            }
        }

        public IObservable<Scan> Data => _scan;
        public IObservable<Unit> OnConnected => _onConnected;
        public IObservable<Unit> OnDisconnected => _onDisconnected;
        public IObservable<Exception> OnError => _onError;

        public LS2Lidar() { }

        public void Connect(string remoteHost, int remotePort)
        {
            if (!IPAddress.TryParse(remoteHost, out IPAddress? address))
            {
                _onError.OnNext(new FormatException($"Invalid IP address: {remoteHost}"));
                return;
            }

            Connect(new IPEndPoint(address, remotePort));
        }

        public void Connect(IPEndPoint remoteEndpoint)
        {
            if (remoteEndpoint is null)
                throw new ArgumentNullException(nameof(remoteEndpoint));

            if (_receiveTask != null)
                Disconnect();

            try
            {
                _socket = new NetSocket(SocketType.Dgram, ProtocolType.Udp)
                {
                    ExclusiveAddressUse = false,
                };

                _socket.Bind(new IPEndPoint(IPAddress.Any, 0));
                _socket.Connect(remoteEndpoint);

                RemoteEndpoint = remoteEndpoint;

                _cancellation = new CancellationTokenSource();
                _receiveTask = Task.Run(() => ReceiveLoop(_socket, _cancellation.Token));

                _isConnected = true; // explicit, not Socket.Connected
                _onConnected.OnNext(Unit.Default);
            }
            catch (Exception ex)
            {
                _socket?.Dispose();
                _socket = null;
                RemoteEndpoint = null;
                _isConnected = false;

                _onError.OnNext(ex);
            }
        }

        public void StartScan() => SendCommand(Protocol.CMD_START_STREAM_DATA);

        public void SendCommand(ReadOnlySpan<byte> command)
        {
            var socket = _socket;
            if (socket is null || !IsConnected)
            {
                _onError.OnNext(new InvalidOperationException("Cannot send: not connected."));
                return;
            }

            try
            {
                socket.Send(command);
            }
            catch (Exception ex)
            {
                _onError.OnNext(ex);
            }
        }

        public void Disconnect()
        {
            bool wasConnected = _isConnected;

            _cancellation?.Cancel();
            _receiveTask?.Wait(500);
            _cancellation?.Dispose();
            _cancellation = null;
            _receiveTask = null;

            _socket?.Close();
            _socket?.Dispose();
            _socket = null;

            RemoteEndpoint = null;
            _isConnected = false;

            if (wasConnected)
                _onDisconnected.OnNext(Unit.Default);
        }

        private async Task ReceiveLoop(NetSocket socket, CancellationToken ct)
        {
            byte[] buffer = ArrayPool<byte>.Shared.Rent(8192);

            try
            {
                while (!ct.IsCancellationRequested)
                {
                    int received = await socket.ReceiveAsync(
                        new ArraySegment<byte>(buffer),
                        SocketFlags.None,
                        ct
                    );

                    if (received <= 0)
                        continue;

                    ProcessDatagram(buffer, received);
                }
            }
            catch (OperationCanceledException)
            { /* expected on Disconnect */
            }
            catch (ObjectDisposedException)
            { /* expected on Disconnect */
            }
            catch (Exception ex)
            {
                _onError.OnNext(ex);
            }
            finally
            {
                ArrayPool<byte>.Shared.Return(buffer);
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
                _parser.Parse(fullPayload, _scanData);

                Interlocked.Exchange(ref _lastPacketTimestamp, Stopwatch.GetTimestamp());

                _scan.OnNext(Scan.Snapshot(_scanData));
            }
        }

        public void Dispose()
        {
            Disconnect();

            _scan.OnCompleted();
            _scan.Dispose();
            _onConnected.OnCompleted();
            _onConnected.Dispose();
            _onDisconnected.OnCompleted();
            _onDisconnected.Dispose();
            _onError.OnCompleted();
            _onError.Dispose();
        }
    }
}
