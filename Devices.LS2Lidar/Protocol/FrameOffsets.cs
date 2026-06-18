namespace Devices.LS2Lidar.Protocol
{
    /// <summary>
    /// Byte offsets for reading specific metadata from the incoming UDP frame header.
    /// </summary>
    public static class FrameOffsets
    {
        public const int HEADER_START = 0;

        /// <summary>High byte of packet length</summary>
        public const int LENGTH_H = 4;

        /// <summary>Low byte of packet length</summary>
        public const int LENGTH_L = 5;

        public const int CHECK_SUM = 6;
        public const int TYPE = 7;
        public const int TOTAL_INDEX_H = 8;
        public const int TOTAL_INDEX_L = 9;
        public const int SUB_PKG_NUM = 10;
        public const int SUB_INDEX = 11;

        /// <summary>The actual LiDAR distance measurements begin at this byte offset.</summary>
        public const int DATA_START = 12;
    }
}
