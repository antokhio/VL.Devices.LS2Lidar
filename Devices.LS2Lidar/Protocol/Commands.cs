using System;

namespace Devices.LS2Lidar.Protocol
{
    public static class Commands
    {
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
        public static ReadOnlySpan<byte> CMD_SET_MAINTENANCE_ACCESS_MODE => CMD_START_STREAM_DATA;

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
    }
}
