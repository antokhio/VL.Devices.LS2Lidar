namespace Devices.LS2Lidar.Protocol
{
    /// <summary>
    /// Used to ignore error states or dead zones emitted by the hardware sensor.
    /// Distance validity is bounded by the physical range declared in <see cref="Sensor"/>
    /// (DEFAULT_MIN_RANGE / DEFAULT_MAX_RANGE). The 50000 "no return" sentinel reported by
    /// the device falls outside that range and is therefore rejected automatically.
    /// </summary>
    public static class ReadingFilters
    {
        /// <summary>
        /// Raw intensity values above this threshold indicate oversaturation/reflection
        /// and should be clamped.
        /// </summary>
        public const int INTENSITY_OVERFLOW_THRESHOLD = 55000;

        /// <summary>
        /// The clamped intensity value to apply when an overflow occurs.
        /// Integer (matching the vendor parser) so the downstream scaling stays
        /// whole-number, e.g. 600 / 25 = 24.
        /// </summary>
        public const int INTENSITY_OVERFLOW_VALUE = 600;
    }
}
