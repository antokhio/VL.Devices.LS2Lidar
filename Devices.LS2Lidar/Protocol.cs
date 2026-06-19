using System;

namespace Devices.LS2Lidar
{
    /// <summary>
    /// Central definition of the LS2 LiDAR wire protocol and fixed hardware specifications.
    /// </summary>
    public static class Protocol
    {
        // --- Commands ---

        /// <summary>
        /// Commands the LiDAR to begin transmitting distance data streams over UDP.
        /// Acts as the primary initialization and wake-up payload for the device.
        /// </summary>
        public static ReadOnlySpan<byte> CMD_START_STREAM_DATA =>
            new byte[] { 0xFA, 0x5A, 0xA5, 0xAA, 0x00, 0x02, 0x01, 0x01 };

        /// <summary>
        /// Instructs the LiDAR to halt the transmission of UDP data streams.
        /// </summary>
        public static ReadOnlySpan<byte> CMD_STOP_STREAM_DATA =>
            new byte[] { 0xFA, 0x5A, 0xA5, 0xAA, 0x00, 0x02, 0x02, 0x02 };

        /// <summary>
        /// Triggers a soft reboot or restart of the LiDAR hardware.
        /// </summary>
        public static ReadOnlySpan<byte> CMD_REBOOT =>
            new byte[] { 0xFA, 0x5A, 0xA5, 0xAA, 0x00, 0x02, 0x03, 0x03 };

        /// <summary>
        /// Requests the current operational status and health metrics from the device.
        /// </summary>
        public static ReadOnlySpan<byte> CMD_READ_DEVICE_STATE =>
            new byte[] { 0xFA, 0x5A, 0xA5, 0xAA, 0x00, 0x02, 0x04, 0x04 };

        /// <summary>
        /// Requests the factory serial number of the LiDAR unit.
        /// </summary>
        public static ReadOnlySpan<byte> CMD_READ_SERIAL_NUMBER =>
            new byte[] { 0xFA, 0x5A, 0xA5, 0xAA, 0x00, 0x02, 0x05, 0x05 };

        /// <summary>
        /// Commands the device to enter maintenance access mode.
        /// </summary>
        /// <remarks>Shares the exact byte payload as <see cref="CMD_START_STREAM_DATA"/>.</remarks>
        public static ReadOnlySpan<byte> CMD_SET_MAINTENANCE_ACCESS_MODE =>
            CMD_START_STREAM_DATA;

        /// <summary>
        /// Requests the device identification details.
        /// </summary>
        /// <remarks>Shares the exact byte payload as <see cref="CMD_START_STREAM_DATA"/>.</remarks>
        public static ReadOnlySpan<byte> CMD_READ_IDENTIFY => CMD_START_STREAM_DATA;

        /// <summary>
        /// Requests the current firmware version installed on the device.
        /// </summary>
        /// <remarks>Shares the exact byte payload as <see cref="CMD_START_STREAM_DATA"/>.</remarks>
        public static ReadOnlySpan<byte> CMD_READ_FIRMWARE_VERSION => CMD_START_STREAM_DATA;

        /// <summary>
        /// The fixed byte length (8 bytes) of all control commands sent to the LiDAR.
        /// </summary>
        public const int SIZE_OF_CMD = 8;

        /// <summary>
        /// Byte offsets for reading specific metadata from the incoming UDP frame header.
        /// </summary>
        public static class Offsets
        {
            public const int TOTAL_INDEX_H = 8;
            public const int TOTAL_INDEX_L = 9;
            public const int SUB_PKG_NUM = 10;
            public const int SUB_INDEX = 11;

            /// <summary>The actual LiDAR distance measurements begin at this byte offset.</summary>
            public const int DATA_START = 12;
        }

        // --- Scan geometry ---

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

        // --- UDP transport ---

        /// <summary>
        /// Maximum payload byte size of a single UDP command data frame.
        /// </summary>
        public const int CMD_FRAME_MAX_LEN = 1500;

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

        // --- Reading filters ---

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
