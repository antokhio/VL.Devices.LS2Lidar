namespace Devices.LS2Lidar.Model
{
    /// <summary>
    /// Represents a single discrete measurement from the LiDAR.
    /// This is a struct (value type) to ensure it lives contiguously in memory within the array.
    /// </summary>
    public struct ScanPoint
    {
        public float Distance;
        public float Angle;
        public float Intensity;
        public bool IsValid;
    }
}
