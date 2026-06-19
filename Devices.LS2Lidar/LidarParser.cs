using System;
using System.Buffers.Binary;
using Devices.LS2Lidar.Model;
using Devices.LS2Lidar.Protocol;

namespace Devices.LS2Lidar
{
    public class LidarParser
    {
        public void Parse(ReadOnlySpan<byte> payload, ScanData destination)
        {
            destination.Reset();

            // The LS2027 streams a sweep as two contiguous big-endian uint16 blocks
            // with NO leading header (verified on-wire: payload == 811*2 + 811*2):
            //   [0    .. N*2)  => intensity block  (N = SCAN_MEASURES_COUNT)
            //   [N*2  .. N*4)  => distance block   (millimetres)
            // A distance-only frame is a single N*2 distance block at offset 0.
            int blockBytes = Sensor.SCAN_MEASURES_COUNT * 2;

            if (payload.Length < blockBytes)
                return;

            // Intensity is present only when the payload carries BOTH blocks.
            // When present, intensity is the FIRST block and distance the SECOND.
            bool hasIntensity = payload.Length >= blockBytes * 2;
            int distanceBase = hasIntensity ? blockBytes : 0;
            const int intensityBase = 0;

            for (int i = 0; i < Sensor.SCAN_MEASURES_COUNT; i++)
            {
                // 1. Distance lives in the SECOND block when intensity is present.
                int distanceOffset = distanceBase + (i * 2);

                if (distanceOffset + 2 > payload.Length)
                    break;

                ushort rawDistance = BinaryPrimitives.ReadUInt16BigEndian(
                    payload.Slice(distanceOffset, 2)
                );

                if (
                    rawDistance < ReadingFilters.RAW_DISTANCE_MIN
                    || rawDistance > ReadingFilters.RAW_DISTANCE_MAX
                )
                {
                    destination.Points[i].IsValid = false;
                    continue;
                }

                // 2. Intensity lives in the FIRST block (raw 0..65535).
                float scaledIntensity = 0f;
                if (hasIntensity)
                {
                    int intensityOffset = intensityBase + (i * 2);
                    ushort rawIntensity = BinaryPrimitives.ReadUInt16BigEndian(
                        payload.Slice(intensityOffset, 2)
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

                // 3. Mutate the pre-allocated struct IN-PLACE (no `new`, zero GC).
                ref ScanPoint point = ref destination.Points[i];
                point.Distance = rawDistance / 1000.0f; // 1 unit = 1mm
                point.Angle =
                    Sensor.DEFAULT_ANGLE_MIN_CYCLES + (i * Sensor.DEFAULT_ANGLE_INCREMENT_CYCLES);
                point.Intensity = scaledIntensity;
                point.IsValid = true;

                destination.ValidPointCount++;
            }
        }
    }
}
