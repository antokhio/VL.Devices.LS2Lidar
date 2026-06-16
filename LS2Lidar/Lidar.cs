using System;
using System.Buffers.Binary;
using System.Runtime.InteropServices;

namespace LS2Lidar
{
    public static class LidarConstants
    {
        public const int MaxCommandFrameLength = 1500;
        public const int PointsPerScan = 811; // range_end (810) - range_start (0) + 1

        public const byte STX = 0xAA;
        public const byte ETX = 0x66;
    }

    public static class LidarCommands
    {
        public static ReadOnlySpan<byte> StartStreamData =>
            new byte[] { 0xFA, 0x5A, 0xA5, 0xAA, 0x00, 0x02, 0x01, 0x01 };
        public static ReadOnlySpan<byte> StopStreamData =>
            new byte[] { 0xFA, 0x5A, 0xA5, 0xAA, 0x00, 0x02, 0x02, 0x02 };
        public static ReadOnlySpan<byte> Reboot =>
            new byte[] { 0xFA, 0x5A, 0xA5, 0xAA, 0x00, 0x02, 0x03, 0x03 };
        public static ReadOnlySpan<byte> SetMaintenanceMode =>
            new byte[] { 0xFA, 0x5A, 0xA5, 0xAA, 0x00, 0x02, 0x01, 0x01 };
        public static ReadOnlySpan<byte> ReadDeviceState =>
            new byte[] { 0xFA, 0x5A, 0xA5, 0xAA, 0x00, 0x02, 0x04, 0x04 };
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public readonly struct CommandFrameHeader
    {
        public readonly byte HeaderStart; // 0
        // Skip unused padding/offsets up to Length_H
        // Offset mapping based on C++ macros (CMD_FRAME_HEADER_START, etc.)
        // Note: It's often safer to read headers procedurally using spans
        // due to network byte order (Big-Endian) conversions.
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public readonly struct SensorDataHeader
    {
        public readonly byte Header;
        public readonly byte CmdId;
        public readonly ushort RangeStart;
        public readonly ushort RangeEnd;
        public readonly byte CheckValue;
        // The flexible array member (sens_data[0]) is handled dynamically via Span<T>
    }

    public ref struct SensorFrame
    {
        public ReadOnlySpan<ushort> Distances { get; }
        public ReadOnlySpan<ushort> Intensities { get; }
        public bool HasIntensity { get; }

        public SensorFrame(ReadOnlySpan<byte> rawPayload)
        {
            // 1. Extract Header (7 bytes based on C++ SensData struct)
            var header = MemoryMarshal.Read<SensorDataHeader>(rawPayload[..7]);

            // 2. Validate Checksum (Sum of all bytes == check_value)
            // Implementation of checksum goes here...

            // 3. Extract Data Slice
            ReadOnlySpan<byte> dataBytes = rawPayload[7..];
            int totalPoints = LidarConstants.PointsPerScan;

            // Check if payload includes intensity data (4 bytes per point vs 2)
            HasIntensity = dataBytes.Length == (totalPoints * 4);

            // 4. Endianness Conversion
            // Since the network is Big-Endian, we must swap the bytes.
            ushort[] decodedData = new ushort[HasIntensity ? totalPoints * 2 : totalPoints];
            for (int i = 0; i < decodedData.Length; i++)
            {
                decodedData[i] = BinaryPrimitives.ReadUInt16BigEndian(dataBytes.Slice(i * 2, 2));
            }

            Distances = new ReadOnlySpan<ushort>(decodedData, 0, totalPoints);

            if (HasIntensity)
            {
                Intensities = new ReadOnlySpan<ushort>(decodedData, totalPoints, totalPoints);
            }
            else
            {
                Intensities = ReadOnlySpan<ushort>.Empty;
            }
        }
    }

    public class LidarParser
    {
        // The C++ driver uses a starting angle of 0xFFF92230.
        // Cast to signed 32-bit int, this is -450000.
        // -450000 / 10000.0 = -45.0 degrees.
        public const double AngleMinRadians = (-45.0 / 180.0 * Math.PI) - (Math.PI / 2.0);

        // Step width is 0xD05 (3333). 3333 / 10000.0 = 0.3333 degrees.
        public const double AngleIncrementRadians = (0.3333 / 180.0 * Math.PI);

        public LidarScanData Parse(SensorFrame frame)
        {
            var distancesMeters = new float[LidarConstants.PointsPerScan];
            var intensityScaled = new float[LidarConstants.PointsPerScan];

            for (int i = 0; i < LidarConstants.PointsPerScan; i++)
            {
                // Distance conversion: Raw value / 100.0f = meters
                distancesMeters[i] = frame.Distances[i] / 100.0f;

                // Intensity scaling logic from C++ driver
                if (frame.HasIntensity)
                {
                    ushort rawIntensity = frame.Intensities[i];
                    if (rawIntensity > 55000)
                    {
                        intensityScaled[i] = 600f;
                    }
                    else if (rawIntensity > 5000)
                    {
                        intensityScaled[i] = 200f + ((rawIntensity - 5000f) / 1200f);
                    }
                    else
                    {
                        intensityScaled[i] = rawIntensity / 25f;
                    }
                }
            }

            return new LidarScanData { Distances = distancesMeters, Intensities = intensityScaled };
        }
    }

    public class LidarScanData
    {
        public float[] Distances { get; set; }
        public float[] Intensities { get; set; }
    }
}
