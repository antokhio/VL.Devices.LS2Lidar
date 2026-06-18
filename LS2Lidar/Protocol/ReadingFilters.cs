namespace Devices.LS2Lidar.Protocol
{
    /// <summary>
    /// Used to ignore error states or dead zones emitted by the hardware sensor.
    /// </summary>
    public static class ReadingFilters
    {
        /// <summary>
        /// Raw 16-bit distance values below this are treated as invalid or error states.
        /// (Original C++ name: INDEX_RANGE_MIN, defined as 20 * 60)
        /// </summary>
        public const int RAW_DISTANCE_MIN = 1200;

        /// <summary>
        /// Raw 16-bit distance values above this are treated as infinite or out-of-bounds.
        /// (Original C++ name: INDEX_RANGE_MAX)
        /// </summary>
        public const int RAW_DISTANCE_MAX = 65415;
    }
}
