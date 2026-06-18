using System;
using System.Buffers.Binary;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace Devices.LS2Lidar
{
    public static class LS2LidarConstants
    {
        public const int MaxCommandFrameLength = 1500;
        public const int PointsPerScan = 811; // range_end (810) - range_start (0) + 1

        // Valid measurement range in meters (matches the reference driver defaults).
        // Readings outside this range are no-return points and should be discarded.
        public const float MinRangeMeters = 0.05f;
        public const float MaxRangeMeters = 10.0f;

        public const byte STX = 0xAA;
        public const byte ETX = 0x66;
    }

    public static class LS2LidarCommands
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
            int totalPoints = LS2LidarConstants.PointsPerScan;

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

    public class LS2LidarParser
    {
        // The C++ driver uses a starting angle of 0xFFF92230.
        // Cast to signed 32-bit int, this is -450000.
        // -450000 / 10000.0 = -45.0 degrees.
        public const double AngleMinRadians = (-45.0 / 180.0 * Math.PI) - (Math.PI / 2.0);

        // Step width is 0xD05 (3333). 3333 / 10000.0 = 0.3333 degrees.
        public const double AngleIncrementRadians = (0.3333 / 180.0 * Math.PI);

        public LS2LidarScanData Parse(SensorFrame frame)
        {
            var points = new ScanPoint[LS2LidarConstants.PointsPerScan];

            for (int i = 0; i < LS2LidarConstants.PointsPerScan; i++)
            {
                // Distance conversion: Raw value / 100.0f = meters
                float distanceMeters = frame.Distances[i] / 100.0f;

                // Angle: starting angle plus per-step increment (radians)
                double angle = AngleMinRadians + (i * AngleIncrementRadians);

                // Intensity scaling logic from C++ driver
                float intensity = 0f;
                if (frame.HasIntensity)
                {
                    ushort rawIntensity = frame.Intensities[i];
                    if (rawIntensity > 55000)
                    {
                        intensity = 600f;
                    }
                    else if (rawIntensity > 5000)
                    {
                        intensity = 200f + ((rawIntensity - 5000f) / 1200f);
                    }
                    else
                    {
                        intensity = rawIntensity / 25f;
                    }
                }

                points[i] = new ScanPoint(distanceMeters, angle, intensity);
            }

            return new LS2LidarScanData(
                points,
                frame.HasIntensity,
                AngleMinRadians,
                AngleIncrementRadians
            );
        }
    }

    public readonly struct ScanPoint
    {
        public ScanPoint(float distance, double angle, float intensity)
        {
            Distance = distance;
            Angle = angle;
            Intensity = intensity;
        }

        // Measured distance in meters.
        public float Distance { get; }

        // Angle of the measurement in radians.
        public double Angle { get; }

        // Scaled intensity; 0 when the frame carries no intensity data.
        public float Intensity { get; }

        // False for no-return readings (e.g. zero distance) that would otherwise
        // collapse onto the sensor origin when drawn.
        public bool IsValid =>
            float.IsFinite(Distance)
            && Distance >= LS2LidarConstants.MinRangeMeters
            && Distance <= LS2LidarConstants.MaxRangeMeters;
    }

    public class LS2LidarScanData
    {
        public LS2LidarScanData(
            IReadOnlyList<ScanPoint> points,
            bool hasIntensity,
            double angleMin,
            double angleIncrement
        )
        {
            Points = points;
            HasIntensity = hasIntensity;
            AngleMin = angleMin;
            AngleIncrement = angleIncrement;
        }

        public IReadOnlyList<ScanPoint> Points { get; }
        public bool HasIntensity { get; }

        public double AngleMin { get; }
        public double AngleIncrement { get; }

        public int Count => Points.Count;
        public double AngleMax => AngleMin + (Count - 1) * AngleIncrement;
    }

    public sealed class LS2LidarFrameAssembler
    {
        // Frame header byte offsets (SDKELI UDP binary protocol).
        private const int LengthH = 4;
        private const int LengthL = 5;
        private const int CheckSum = 6;
        private const int Type = 7;
        private const int TotalIndexH = 8;
        private const int TotalIndexL = 9;
        private const int SubPkgNum = 10;
        private const int SubIndex = 11;
        private const int DataStart = 12;

        private const int MinSubPkgNum = 2;
        private const int MaxSubPkgNum = 4;

        private static readonly byte[] FrameHeader = { 0xFA, 0x5A, 0xA5, 0xAA };

        private readonly SubPackage[] _parts = new SubPackage[MaxSubPkgNum];

        // Reassembles the multi-datagram scan frame; returns the concatenated
        // sensor payload once every sub-package of a frame has been received.
        public bool TryAssemble(ReadOnlySpan<byte> datagram, out byte[] payload)
        {
            payload = Array.Empty<byte>();

            if (datagram.Length < DataStart)
                return false;

            for (int i = 0; i < FrameHeader.Length; i++)
            {
                if (datagram[i] != FrameHeader[i])
                    return false;
            }

            int frameLength = (datagram[LengthH] << 8) | datagram[LengthL];
            int rawLength = frameLength - (DataStart - CheckSum);

            if (rawLength <= 0 || DataStart + rawLength > datagram.Length)
                return false;

            // Checksum spans the header type byte through the payload.
            byte checksum = 0;
            int checkCount = rawLength + (DataStart - Type);
            for (int i = 0; i < checkCount; i++)
                checksum += datagram[Type + i];

            if (checksum != datagram[CheckSum])
                return false;

            uint totalIndex = (uint)((datagram[TotalIndexH] << 8) | datagram[TotalIndexL]);
            byte subPackageCount = datagram[SubPkgNum];
            byte subPackageIndex = datagram[SubIndex];

            if (subPackageCount < MinSubPkgNum || subPackageCount > MaxSubPkgNum)
                return false;

            if (subPackageIndex >= MaxSubPkgNum)
                return false;

            ref SubPackage part = ref _parts[subPackageIndex];
            part.TotalIndex = totalIndex;
            part.Index = subPackageIndex;
            part.Length = rawLength;
            if (part.Data is null || part.Data.Length < rawLength)
                part.Data = new byte[rawLength];
            datagram.Slice(DataStart, rawLength).CopyTo(part.Data);

            // All sub-packages must belong to the same frame and be consecutive.
            for (int i = 0; i < subPackageCount - 1; i++)
            {
                if (
                    _parts[i].TotalIndex != _parts[i + 1].TotalIndex
                    || _parts[i].Index + 1 != _parts[i + 1].Index
                )
                {
                    return false;
                }
            }

            int total = 0;
            for (int i = 0; i < subPackageCount; i++)
            {
                if (_parts[i].Data is null)
                    return false;
                total += _parts[i].Length;
            }

            var buffer = new byte[total];
            int offset = 0;
            for (int i = 0; i < subPackageCount; i++)
            {
                Array.Copy(_parts[i].Data!, 0, buffer, offset, _parts[i].Length);
                offset += _parts[i].Length;
            }

            payload = buffer;
            return true;
        }

        private struct SubPackage
        {
            public uint TotalIndex;
            public byte Index;
            public int Length;
            public byte[]? Data;
        }
    }
}
