namespace Devices.LS2Lidar.Model
{
    /// <summary>
    /// A pre-allocated, mutable buffer for a complete 360-degree sweep.
    /// In a real-time environment (VL), you instantiate this ONCE and pass it to the parser
    /// repeatedly; the parser overwrites it in place to avoid garbage collection.
    /// </summary>
    /// <remarks>
    /// Because this buffer is reused and mutated every frame, it must NOT be retained, queued,
    /// or shared across threads. To keep a sweep beyond the current frame, take an immutable
    /// <see cref="Scan"/> snapshot via <see cref="Scan.Snapshot(ScanData)"/>.
    /// </remarks>
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
