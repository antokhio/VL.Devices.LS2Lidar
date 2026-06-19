namespace Devices.LS2Lidar.Model
{
    /// <summary>
    /// Represents a single discrete measurement from the LiDAR.
    /// This is a readonly value type to ensure it lives contiguously in memory within the array
    /// and cannot be mutated after it has been written into a buffer slot.
    /// </summary>
    public readonly struct ScanPoint
    {
        public ScanPoint(float distance, float angle, float intensity, bool isValid)
        {
            Distance = distance;
            Angle = angle;
            Intensity = intensity;
            IsValid = isValid;
        }

        public float Distance { get; }
        public float Angle { get; }
        public float Intensity { get; }
        public bool IsValid { get; }
    }
}
