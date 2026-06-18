namespace Devices.LS2Lidar.Protocol
{
    /// <summary>
    /// Protocol-level byte markers used to frame boundaries.
    /// </summary>
    public static class Markers
    {
        /// <summary>
        /// Start of Text: Indicates the beginning of a sequence or frame.
        /// </summary>
        public const byte STX = 0xAA;

        /// <summary>
        /// End of Text: Indicates the end of a sequence or frame.
        /// </summary>
        public const byte ETX = 0x66;
    }
}
