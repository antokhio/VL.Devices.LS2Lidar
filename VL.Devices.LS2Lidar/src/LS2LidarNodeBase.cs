using System.Reactive.Disposables;
using VL.Core;
using VL.Core.Import;
using VL.Lang;
using VL.Lib.Basics.Resources;

namespace VL.Devices.LS2Lidar
{
    /// <summary>
    /// VL warning handling base class.
    /// </summary>
    [ProcessNode]
    public abstract class LS2LidarNodeBase : IDisposable
    {
        private readonly NodeContext _nodeContext;
        private readonly IVLRuntime _runtime;

        public LS2LidarNodeBase(NodeContext nodeContext)
        {
            _nodeContext = nodeContext;
            _runtime = IVLRuntime.Current!;
        }

        protected void Warn(string message, int delayDisposalInMilliseconds = 1000)
        {
            ResourceProvider
                .NewPooledSystemWide(
                    _nodeContext.Path,
                    _ =>
                    {
                        var messages = new CompositeDisposable();
                        foreach (var id in _nodeContext.Path.Stack)
                        {
                            _runtime
                                .AddPersistentMessage(
                                    new Message(id, MessageSeverity.Warning, message)
                                )
                                .DisposeBy(messages);
                        }

                        return messages;
                    },
                    delayDisposalInMilliseconds
                )
                .GetHandle()
                .Dispose();
        }

        public virtual void Dispose() { }
    }
}
