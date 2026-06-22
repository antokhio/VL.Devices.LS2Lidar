using VL.Core;
using VL.Core.Import;

namespace VL.Devices.LS2Lidar
{
    [ProcessNode(Name = "LS2Lidar")]
    public class LS2LidarNode : LS2LidarNodeBase, IDisposable
    {
        public const int WarningDisposalDelayMs = 8000;

        private readonly LS2Lidar _lidar = new();

        private string _remoteHost = LS2Lidar.DefaultRemoteHost;
        private int _remotePort = LS2Lidar.DefaultRemotePort;
        private bool _scan = true;

        // Tracks the endpoint we actually told the core to connect to, so we only
        // reconnect when the requested host/port differs from what's live.
        private string? _connectedHost;
        private int? _connectedPort;

        private readonly IDisposable _errorSubscription;

        public ILS2Lidar Output => _lidar;
        public bool IsConnected => _lidar.IsConnected;
        public bool IsScanning => _lidar.IsScanning;

        public LS2LidarNode([Pin(Visibility = Model.PinVisibility.Hidden)] NodeContext nodeContext)
            : base(nodeContext)
        {
            _errorSubscription = _lidar.OnError.Subscribe(ex =>
                Warn(ex.Message, WarningDisposalDelayMs)
            );
        }

        public void SetConnection(
            string remoteHost = LS2Lidar.DefaultRemoteHost,
            int remotePort = LS2Lidar.DefaultRemotePort
        )
        {
            _remoteHost = remoteHost;
            _remotePort = remotePort;
        }

        public void SetScan(bool scan = true)
        {
            _scan = scan;
        }

        /// <summary>
        /// Per-frame entry point. Brings the underlying <see cref="LS2Lidar"/> in line with the
        /// node's desired state: when <paramref name="isEnabled"/> is <see langword="false"/> the
        /// device is disconnected; otherwise it (re)connects to the configured endpoint and starts
        /// streaming when scanning was requested. Idempotent — designed to be called every frame.
        /// </summary>
        public void Update(bool isEnabled = true)
        {
            if (!isEnabled)
            {
                if (_lidar.IsConnected)
                {
                    _lidar.Disconnect();
                    _connectedHost = null;
                    _connectedPort = null;
                }
                return;
            }

            // Enabled: (re)connect if the target endpoint changed or we're not connected
            // (the latter also auto-recovers if the device dropped between frames).
            var endpointChanged = _remoteHost != _connectedHost || _remotePort != _connectedPort;

            if (endpointChanged || !_lidar.IsConnected)
            {
                _lidar.Connect(_remoteHost, _remotePort);

                // A bad host pushes through OnError and leaves IsConnected false.
                if (_lidar.IsConnected)
                {
                    _connectedHost = _remoteHost;
                    _connectedPort = _remotePort;
                }
                else
                {
                    _connectedHost = null;
                    _connectedPort = null;
                    return; // nothing to scan if we didn't connect
                }
            }

            // Kick off streaming once when requested and connected.
            if (_scan && !_lidar.IsScanning)
                _lidar.StartScan();
        }

        public override void Dispose()
        {
            _errorSubscription.Dispose();
            _lidar.Dispose();
            base.Dispose();
        }
    }
}
