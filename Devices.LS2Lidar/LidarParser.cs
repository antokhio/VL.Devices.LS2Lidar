using System;
using System.Buffers.Binary;
using Devices.LS2Lidar.Model;

namespace Devices.LS2Lidar
{
    public class LidarParser
    {
        public void Parse(ReadOnlySpan<byte> payload, ScanData destination)
        {
            destination.Reset();

            // The LS2027 streams a sweep as two contiguous big-endian uint16 blocks
            // with NO leading header. Block order verified against the vendor SDK
            // (sdkeli_ls_sensor_frame.cpp: GetSensDataOfIndex => sens_data[i];
            //  GetSensIntensityOfIndex => sens_data[N + i]):
            //   [0    .. N*2)  => distance block   (millimetres, N = SCAN_MEASURES_COUNT)
            //   [N*2  .. N*4)  => intensity block  (raw reflection strength)
            // A distance-only frame is a single N*2 distance block at offset 0.
            int blockBytes = Protocol.SCAN_MEASURES_COUNT * 2;

            if (payload.Length < blockBytes)
                return;

            // Intensity is present only when the payload carries BOTH blocks.
            // Distance is ALWAYS the first block; intensity (when present) is the second.
            bool hasIntensity = payload.Length >= blockBytes * 2;
            const int distanceBase = 0;
            int intensityBase = blockBytes;

            for (int i = 0; i < Protocol.SCAN_MEASURES_COUNT; i++)
            {
                // 1. Distance lives in the FIRST block.
                int distanceOffset = distanceBase + (i * 2);

                if (distanceOffset + 2 > payload.Length)
                    break;

                ushort rawDistance = BinaryPrimitives.ReadUInt16BigEndian(
                    payload.Slice(distanceOffset, 2)
                );

                // Distance is in millimetres. Reject returns outside the sensor's
                // physical range: 0 / sub-minimum readings and the 50000 "no return"
                // sentinel (50 m) both fall outside [MIN_RANGE, MAX_RANGE].
                float distanceMeters = rawDistance / 1000.0f;
                if (
                    distanceMeters < Protocol.DEFAULT_MIN_RANGE
                    || distanceMeters > Protocol.DEFAULT_MAX_RANGE
                )
                {
                    destination.Points[i].IsValid = false;
                    continue;
                }

                // 2. Intensity lives in the SECOND block (raw reflection strength).
                //    The vendor parser (sdkeli_ls1207de_parser.cpp) operates on an
                //    `unsigned short`, so every step is INTEGER arithmetic and always
                //    yields whole numbers (e.g. raw 1725 => 69), matching the values
                //    shown by the vendor tool.
                float scaledIntensity = 0f;
                if (hasIntensity)
                {
                    int intensityOffset = intensityBase + (i * 2);
                    int rawIntensity = BinaryPrimitives.ReadUInt16BigEndian(
                        payload.Slice(intensityOffset, 2)
                    );

                    // Saturated returns are clamped to 600 first; the vendor then runs
                    // that clamped value back through the scaler (separate `if` blocks).
                    if (rawIntensity > Protocol.INTENSITY_OVERFLOW_THRESHOLD)
                    {
                        rawIntensity = Protocol.INTENSITY_OVERFLOW_VALUE;
                    }

                    scaledIntensity =
                        rawIntensity > 5000
                            ? 200 + ((rawIntensity - 5000) / 1200)
                            : rawIntensity / 25;
                }

                // 3. Mutate the pre-allocated struct IN-PLACE (no `new`, zero GC).
                ref ScanPoint point = ref destination.Points[i];
                point.Distance = distanceMeters;
                point.Angle =
                    Protocol.DEFAULT_ANGLE_MIN_CYCLES
                    + (i * Protocol.DEFAULT_ANGLE_INCREMENT_CYCLES);
                point.Intensity = scaledIntensity;
                point.IsValid = true;

                destination.ValidPointCount++;
            }
        }
    }
}
