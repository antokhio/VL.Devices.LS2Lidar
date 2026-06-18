using System;
using System.Buffers.Binary;
using Devices.LS2Lidar.Model;
using Devices.LS2Lidar.Protocol;

namespace Devices.LS2Lidar
{
    public class LidarParser
    {
        // The C++ driver defines the sensor header as exactly 7 bytes long
        private const int SENSOR_HEADER_LENGTH = 7;

        public void Parse(ReadOnlySpan<byte> payload, LidarScanData destination)
        {
            destination.Reset();

            if (payload.Length <= SENSOR_HEADER_LENGTH)
                return;

            // Strip the 7-byte sensor header to align purely with the data blocks
            ReadOnlySpan<byte> dataBytes = payload.Slice(SENSOR_HEADER_LENGTH);

            // Calculate if intensity data is attached based on the remaining payload size.
            // Distance alone is 811 * 2 = 1622 bytes. Intensity adds another 1622 bytes.
            int expectedDistanceBytes = Sensor.SCAN_MEASURES_COUNT * 2;
            bool hasIntensity = dataBytes.Length >= expectedDistanceBytes * 2;

            for (int i = 0; i < Sensor.SCAN_MEASURES_COUNT; i++)
            {
                // 1. Calculate the byte offset for this specific point's Distance
                int distanceOffset = i * 2;

                if (distanceOffset + 2 > dataBytes.Length)
                    break;

                ushort rawDistance = BinaryPrimitives.ReadUInt16BigEndian(
                    dataBytes.Slice(distanceOffset, 2)
                );

                if (
                    rawDistance < ReadingFilters.RAW_DISTANCE_MIN
                    || rawDistance > ReadingFilters.RAW_DISTANCE_MAX
                )
                {
                    destination.Points[i].IsValid = false;
                    continue;
                }

                // 2. Calculate Intensity if available
                float scaledIntensity = 0f;
                if (hasIntensity)
                {
                    // The intensity block starts immediately AFTER the entire distance block
                    int intensityOffset = expectedDistanceBytes + (i * 2);
                    ushort rawIntensity = BinaryPrimitives.ReadUInt16BigEndian(
                        dataBytes.Slice(intensityOffset, 2)
                    );

                    // Scale logic ported directly from the C++ driver
                    if (rawIntensity > ReadingFilters.INTENSITY_OVERFLOW_THRESHOLD)
                    {
                        scaledIntensity = ReadingFilters.INTENSITY_OVERFLOW_VALUE;
                    }
                    else if (rawIntensity > 5000)
                    {
                        scaledIntensity = 200f + ((rawIntensity - 5000f) / 1200f);
                    }
                    else
                    {
                        scaledIntensity = rawIntensity / 25f;
                    }
                }

                // 3. Write directly into the pre-allocated struct array
                destination.Points[i] = new ScanPoint
                {
                    Distance = rawDistance / 1000.0f, // Assuming 1 unit = 1mm
                    Angle =
                        Sensor.DEFAULT_ANGLE_MIN_CYCLES
                        + (i * Sensor.DEFAULT_ANGLE_INCREMENT_CYCLES),
                    Intensity = scaledIntensity,
                    IsValid = true,
                };

                destination.ValidPointCount++;
            }
        }
    }
}
