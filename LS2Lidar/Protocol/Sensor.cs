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

        /// <summary>
        /// The default starting angle of the LiDAR sweep cycles (0.0 to 1.0 full circle).
        /// Represents -135 degrees.
        /// </summary>
        public const float DEFAULT_ANGLE_MIN_CYCLES = -0.375f;

        /// <summary>
        /// The angle increment between each discrete measurement mapped to vvvv cycles.
        /// Represents exactly 1/3 of a degree (1.0 / 1080.0).
        /// </summary>
        public const float DEFAULT_ANGLE_INCREMENT_CYCLES = 0.0009259259f;
    }
}
