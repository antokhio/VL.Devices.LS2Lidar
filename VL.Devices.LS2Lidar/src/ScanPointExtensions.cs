using Devices.LS2Lidar;
using Stride.Core.Mathematics;

namespace VL.Devices.LS2Lidar
{
    public static class ScanPointExtensions
    {
        // Converts a polar scan point to Cartesian coordinates where 1 unit equals 1 meter.
        public static Vector2 ToVector(this ScanPoint point)
        {
            float x = (float)(-point.Distance * Math.Sin(point.Angle));
            float y = (float)(point.Distance * Math.Cos(point.Angle));
            return new Vector2(x, y);
        }
    }
}
