namespace Devices.LS2Lidar.Protocol
{
    /// <summary>
    /// Defines the physical boundaries and default parameters for a single 360-degree sweep.
    /// </summary>
    public static class Sensor
    {
        /// <summary>
        /// Defines the zero-based starting array index for the sequence of data points.
        /// </summary>
        public const int SCAN_START_INDEX = 0;

        /// <summary>
        /// Defines the final array index for the data points.
        /// </summary>
        public const int SCAN_END_INDEX = 810;

        /// <summary>
        /// Defines the total count of discrete measurements per scan.
        /// </summary>
        public const int SCAN_MEASURES_COUNT = SCAN_END_INDEX - SCAN_START_INDEX + 1;

        /// <summary>
        /// Default minimum valid physical distance reading in meters.
        /// </summary>
        public const float DEFAULT_MIN_RANGE = 0.05f;

        /// <summary>
        /// Default maximum valid physical distance reading in meters.
        /// </summary>
        public const float DEFAULT_MAX_RANGE = 20.0f;
    }
}
