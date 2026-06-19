namespace Devices.LS2Lidar.Model
{
    /// <summary>
    /// A pre-allocated container for a complete 360-degree sweep.
    /// In a real-time environment (VL), you instantiate this ONCE and pass it to the parser repeatedly.
    /// </summary>
    public class ScanData
    {
        /// <summary>
        /// The fixed-size array of measurements.
        /// Pre-allocated to the exact size of a scan to avoid garbage collection.
        /// </summary>
        public readonly ScanPoint[] Points;

        /// <summary>
        /// How many valid points were actually recorded in the most recent scan.
        /// </summary>
        public int ValidPointCount { get; set; }

        public ScanData()
        {
            // Allocate the array ONCE based on our Protocol constants (811 points)
            Points = new ScanPoint[Protocol.SCAN_MEASURES_COUNT];
        }

        /// <summary>
        /// Quickly marks all points as invalid before a new parsing pass begins.
        /// </summary>
        public void Reset()
        {
            ValidPointCount = 0;
            // Since Points is a pre-allocated array, we don't re-instantiate it.
            // The parser will just overwrite the values.
        }
    }
}
