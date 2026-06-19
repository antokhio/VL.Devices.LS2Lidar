using System;
using System.Collections.Generic;

namespace Devices.LS2Lidar.Model
{
    /// <summary>
    /// An immutable snapshot of a single 360-degree sweep.
    /// </summary>
    /// <remarks>
    /// Unlike <see cref="ScanData"/> — a reusable buffer that the parser mutates in place — a
    /// <see cref="Scan"/> never changes after creation, so it is safe to retain, queue, sample,
    /// or hand across threads. Only the measurements the parser marked valid are captured; each
    /// <see cref="ScanPoint"/> self-describes its angle, so the points are stored packed (no gaps).
    /// </remarks>
    public sealed class Scan
    {
        private readonly IReadOnlyList<ScanPoint> _points;

        private Scan(IReadOnlyList<ScanPoint> points)
        {
            _points = points;
        }

        /// <summary>
        /// The valid measurements of this sweep, in increasing angular order.
        /// </summary>
        public IReadOnlyList<ScanPoint> Points => _points;

        /// <summary>
        /// The number of valid measurements captured in this sweep.
        /// </summary>
        public int Count => _points.Count;

        /// <summary>A shared, empty sweep (no valid measurements).</summary>
        public static Scan Empty { get; } = new Scan(Array.AsReadOnly(Array.Empty<ScanPoint>()));

        /// <summary>
        /// Creates an immutable snapshot from a reusable <see cref="ScanData"/> buffer, copying only
        /// the <see cref="ScanData.ValidPointCount"/> points the parser marked valid.
        /// </summary>
        /// <param name="source">The reusable buffer to copy from. It is not modified or retained.</param>
        public static Scan Snapshot(ScanData source)
        {
            if (source is null)
                throw new ArgumentNullException(nameof(source));

            int validCount = source.ValidPointCount;
            if (validCount <= 0)
                return Empty;

            ScanPoint[] src = source.Points;
            var points = new ScanPoint[validCount];

            // Valid points are interspersed by angular index in the buffer; pack them.
            int written = 0;
            for (int i = 0; i < src.Length && written < validCount; i++)
            {
                if (src[i].IsValid)
                    points[written++] = src[i];
            }

            // Defensive: expose exactly the points written (no-op when written == validCount).
            if (written != points.Length)
                Array.Resize(ref points, written);

            // Wrap so the array cannot be mutated through the exposed IReadOnlyList facade.
            return new Scan(Array.AsReadOnly(points));
        }
    }
}
