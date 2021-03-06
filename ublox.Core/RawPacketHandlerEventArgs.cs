﻿using System;

namespace ublox.Core
{
    public class RawPacketHandlerEventArgs : EventArgs
    {
        internal RawPacketHandlerEventArgs(MessageId messageId, PacketPayload packetPayload)
        {
            MessageId = messageId;
            PacketPayload = packetPayload;
        }
    
        public MessageId MessageId { get; }
        public PacketPayload PacketPayload { get; }
    }
}
