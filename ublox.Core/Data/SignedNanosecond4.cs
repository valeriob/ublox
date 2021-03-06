﻿using System;
using BinarySerialization;

namespace ublox.Core.Data
{
    public class SignedNanosecond4
    {
        public int Value { get; set; }

        [Ignore]
        public TimeSpan Time
        {
            get => TimeSpan.FromTicks(Value * Constants.TicksPerNanosecond);
            set => Value = (int)(value.Ticks / (double)Constants.TicksPerNanosecond);
        }
    }
}
