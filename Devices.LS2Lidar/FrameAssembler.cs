using System;

namespace Devices.LS2Lidar
{
    /// <summary>
    /// A stateless, zero-allocation state machine that stitches UDP sub-packages
    /// together into a complete LiDAR frame payload.
    /// </summary>
    public class FrameAssembler
    {
        // Pre-allocated buffers to completely eliminate GC pressure
        private readonly byte[][] _subBuffers;
        private readonly int[] _subLengths;
        private readonly byte[] _finalPayloadBuffer;

        // State tracking for UDP drops and sequences
        private int _currentTotalIndex = -1;
        private int _expectedSubPkgCount = 0;
        private int _receivedCount = 0;

        public FrameAssembler()
        {
            // Allocate memory EXACTLY ONCE during initialization
            _subBuffers = new byte[Protocol.MAX_SUB_PKG_NUM][];
            _subLengths = new int[Protocol.MAX_SUB_PKG_NUM];

            for (int i = 0; i < Protocol.MAX_SUB_PKG_NUM; i++)
            {
                _subBuffers[i] = new byte[Protocol.CMD_FRAME_MAX_LEN];
            }

            _finalPayloadBuffer = new byte[
                Protocol.MAX_SUB_PKG_NUM * Protocol.CMD_FRAME_MAX_LEN
            ];
        }

        /// <summary>
        /// Attempts to assemble a UDP datagram into a full frame.
        /// </summary>
        /// <returns>True if a full frame was completed on this datagram, false otherwise.</returns>
        public bool TryAssemble(ReadOnlySpan<byte> datagram, out ReadOnlySpan<byte> fullPayload)
        {
            fullPayload = default;

            if (datagram.Length < Protocol.Offsets.DATA_START)
                return false;

            int totalIndex =
                (datagram[Protocol.Offsets.TOTAL_INDEX_H] << 8)
                | datagram[Protocol.Offsets.TOTAL_INDEX_L];
            int subPkgCount = datagram[Protocol.Offsets.SUB_PKG_NUM];
            int subIndex = datagram[Protocol.Offsets.SUB_INDEX];

            // 1. Strict Boundary Validation
            if (
                subPkgCount < Protocol.MIN_SUB_PKG_NUM
                || subPkgCount > Protocol.MAX_SUB_PKG_NUM
            )
                return false;
            if (subIndex < 0 || subIndex >= subPkgCount)
                return false;

            // 2. Handle Dropped UDP Packets (The Resynchronization)
            if (totalIndex != _currentTotalIndex)
            {
                // We received a piece of a NEW frame before finishing the old one.
                // This means an older packet was dropped by the network. Flush the cache!
                ResetState(totalIndex, subPkgCount);
            }

            // 3. Prevent Duplicate Packets
            // (UDP occasionally duplicates packets, which would falsely trigger completion)
            if (_subLengths[subIndex] > 0)
            {
                return false;
            }

            // 4. Extract Payload (Strip Header and Buffer It)
            int dataLength = datagram.Length - Protocol.Offsets.DATA_START;

            // Slice the incoming span and copy it directly into our 2D pre-allocated array
            datagram.Slice(Protocol.Offsets.DATA_START, dataLength).CopyTo(_subBuffers[subIndex]);

            _subLengths[subIndex] = dataLength;
            _receivedCount++;

            // 5. Check for Frame Completion
            if (_receivedCount == _expectedSubPkgCount)
            {
                fullPayload = AssembleFinalPayload();
                return true;
            }

            return false;
        }

        private void ResetState(int newTotalIndex, int newSubPkgCount)
        {
            _currentTotalIndex = newTotalIndex;
            _expectedSubPkgCount = newSubPkgCount;
            _receivedCount = 0;

            // Fast array clear ensures we don't accidentally read stale lengths
            Array.Clear(_subLengths, 0, _subLengths.Length);
        }

        private ReadOnlySpan<byte> AssembleFinalPayload()
        {
            int offset = 0;
            for (int i = 0; i < _expectedSubPkgCount; i++)
            {
                int len = _subLengths[i];
                if (len == 0)
                    continue;

                // Stitch the pre-allocated sub-buffers into the flat final buffer
                Array.Copy(_subBuffers[i], 0, _finalPayloadBuffer, offset, len);
                offset += len;
            }

            // Return a safe span pointing ONLY to the valid data, ignoring trailing empty bytes
            return new ReadOnlySpan<byte>(_finalPayloadBuffer, 0, offset);
        }
    }
}
