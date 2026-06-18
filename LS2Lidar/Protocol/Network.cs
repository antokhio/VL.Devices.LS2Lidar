namespace Devices.LS2Lidar.Protocol
{
    /// <summary>
    /// Defines memory allocation, buffers, and package size constraints for UDP transport.
    /// </summary>
    public static class Network
    {
        /// <summary>
        /// Maximum payload byte size of a single UDP command data frame.
        /// </summary>
        public const int CMD_FRAME_MAX_LEN = 1500;

        /// <summary>
        /// The expected full byte size of a standard LiDAR data frame.
        /// </summary>
        public const int FRAME_LENGTH = 1622;

        /// <summary>
        /// Size allocated for the UDP listener buffer.
        /// </summary>
        public const int RECV_BUFFER_SIZE = 65536;

        /// <summary>
        /// The maximum number of UDP sub-packets that make up one complete 360-degree LiDAR scan.
        /// </summary>
        public const int MAX_SUB_PKG_NUM = 4;

        /// <summary>
        /// The minimum number of UDP sub-packets that make up a scan.
        /// </summary>
        public const int MIN_SUB_PKG_NUM = 2;
    }
}
