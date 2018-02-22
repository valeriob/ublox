using System;
using System.Diagnostics;
using System.IO.Ports;
using System.Threading;
using ublox.Core;

namespace TestClient
{
    class Program
    {
        static void Main(string[] args)
        {
            var serial = new SerialPort("COM3", 9600, Parity.None, 8, StopBits.One);
            var device = new Device(new UbloxSerialPortDeviceWindows(serial));
            device.PositionVelocityTimeUpdated += (from, ev) =>
            {
                Console.WriteLine($"{ev.DateTime.TimeOfDay}   {DateTime.UtcNow.TimeOfDay:h\\:mm\\:ss}   {ev.SatelliteCount}     {ev.Latitude} - {ev.Longitude}");
            };

            serial.Open();

            //device.StartListeningAsync();
            //device.ConfigurePortAsync(ublox.Core.Messages.Enums.UartPort.Usb, 19200,
            //    ublox.Core.Messages.Enums.PortInProtocols.Ubx | ublox.Core.Messages.Enums.PortInProtocols.Nmea,
            //    ublox.Core.Messages.Enums.PortOutProtocols.Ubx).GetAwaiter().GetResult();

            var source = new CancellationTokenSource();
            source.Cancel();
            ListenLoop(source, new UbloxSerialPortDeviceWindows(serial));


            Console.ReadLine();

        }

        public static void Read(SerialPort serial)
        {

        }

        public static async void ListenLoop(CancellationTokenSource cancellationTokenSource, ISerialDevice serialDevice)
        {
            var cancellationToken = cancellationTokenSource.Token;

            do
            {
                using (var stream = new SerialDeviceStream(serialDevice))
                {
                    var sync1 = new byte[1];
                    var sync2 = new byte[1];

                    while (sync2[0] != 0x62)
                    {
                        while (sync1[0] != 0xb5)
                        {
                            await stream.ReadAsync(sync1, 0, sync1.Length, cancellationToken);
                        }

                        await stream.ReadAsync(sync2, 0, sync2.Length, cancellationToken);
                    }


                    var id = new byte[2];
                    var lengthBuffer = new byte[2];

                    await stream.ReadAsync(id, 0, id.Length, cancellationToken);
                    await stream.ReadAsync(lengthBuffer, 0, lengthBuffer.Length, cancellationToken);

                    var length = BitConverter.ToUInt16(lengthBuffer, 0);
                    var payload = new byte[length];
                    await stream.ReadAsync(payload, 0, payload.Length, cancellationToken);

                    var checksum = new byte[2];
                    await stream.ReadAsync(checksum, 0, checksum.Length, cancellationToken);

                    var rawMessage = new UbloxBinaryMessage(id, lengthBuffer, payload, checksum);
                    if (rawMessage.IsChecksumValid())
                    {
                        var msg = rawMessage.ToKnownMessage() as ublox.Core.Messages.NavPvt;
                        if (msg != null)
                            Console.WriteLine($"{msg.UbloxDateTime.DateTime}   {DateTime.UtcNow.TimeOfDay:h\\:mm\\:ss}   {msg.SatelliteCount}     {msg.Latitude} - {msg.Longitude}");
                    }
                }
            } while (!cancellationToken.IsCancellationRequested);
        }

    }

    public class UbloxBinaryMessage
    {
        byte[] _pre;

        public MessageId Id { get; set; }
        public ushort Length { get; set; }
        public byte[] Payload { get; set; }
        public ushort Checksum { get; set; }


        public UbloxBinaryMessage(byte[] id, byte[] length, byte[] payload, byte[] checksum)
        {
            _pre = new byte[4];
            _pre[0] = id[0];
            _pre[1] = id[1];
            _pre[2] = length[0];
            _pre[3] = length[1];
            Id = (MessageId)BitConverter.ToUInt16(new[] { id[1], id[0] }, 0);
            Length = BitConverter.ToUInt16(length, 0);
            Payload = payload;
            Checksum = BitConverter.ToUInt16(checksum, 0);
        }


        ushort ComputeCheckSum()
        {
            int sum1 = 0;
            int sum2 = 0;

            for (int i = 0; i < _pre.Length; i++)
            {
                sum1 += _pre[i];
                sum2 += sum1;
            }

            for (int i = 0; i < Payload.Length; i++)
            {
                sum1 += Payload[i];
                sum2 += sum1;
            }

            var a = sum1 & 0xff;
            var b = sum2 & 0xff;
            return BitConverter.ToUInt16(new[] { (byte)a, (byte)b }, 0);
        }

        internal bool IsChecksumValid()
        {
            var checksum = ComputeCheckSum();
            return checksum == Checksum;
        }

        int _1e7 = 10000000;
        int _1e5 = 10000000;

        public ublox.Core.Messages.NavPvt ToNavPvt()
        {
            var gpsTimeOfWeek = BitConverter.ToUInt32(Payload, 0);

            var anno = BitConverter.ToUInt16(Payload, 4);
            var mese = Payload[6];
            var giorno = Payload[7];
            var ora = Payload[8];
            var minuti = Payload[9];
            var secondi = Payload[10];

            var validity = Payload[11];
            var accuracy = BitConverter.ToUInt32(Payload, 12);
            var nanoseconds = BitConverter.ToInt32(Payload, 16);

            var fixType = Payload[20];
            var flags = Payload[21];
            var flags2 = Payload[22];
            var numsv = Payload[23];

            var lon = (double)BitConverter.ToInt32(Payload, 24) / _1e7;
            var lat = (double)BitConverter.ToInt32(Payload, 28) / _1e7;
            var height = BitConverter.ToInt32(Payload, 32);
            var hMSL = BitConverter.ToInt32(Payload, 36);
            var hAcc = BitConverter.ToUInt32(Payload, 40);

            var vAcc = BitConverter.ToUInt32(Payload, 44);
            var velN = BitConverter.ToInt32(Payload, 48);
            var velE = BitConverter.ToInt32(Payload, 52);
            var velD = BitConverter.ToInt32(Payload, 56);
            var gSpeed = BitConverter.ToInt32(Payload, 60);
            var headMot = (double)BitConverter.ToInt32(Payload, 64) / _1e5;

            var sAcc = BitConverter.ToUInt32(Payload, 68);
            var headaAcc = (double)BitConverter.ToUInt32(Payload, 72) / _1e5;

            var pDOP = BitConverter.ToUInt16(Payload, 76) * 0.01;
            var headVeh = BitConverter.ToInt32(Payload, 84) / _1e5;
            var magDec = BitConverter.ToUInt16(Payload, 88) * 0.01;
            var magAcc = BitConverter.ToUInt16(Payload, 90) * 0.01;

            return new ublox.Core.Messages.NavPvt
            {
                GpsTimeOfWeek = new ublox.Core.Data.GpsTimeOfWeek(TimeSpan.FromMilliseconds(gpsTimeOfWeek)),
                UbloxDateTime = new ublox.Core.Data.UbloxDateTime(new DateTime(anno, mese, giorno, ora, minuti, secondi)),//, validity, accuracy, nanoseconds),
                FixType = (ublox.Core.Messages.Enums.GnssFixType)fixType,
                Flags = flags,
                Flags2 = flags2,
                SatelliteCount = numsv,
                Longitude = lon,
                Latitude = lat,
                Height = new ublox.Core.Data.SignedDistance(height / 1000),
                HeightAboveMeanSeaLevel = new ublox.Core.Data.SignedDistance(hMSL / 1000),
                HorizontalAccuracyEstimate = new ublox.Core.Data.UnsignedDistance(hAcc / 1000),
                VerticalAccuracyEstimate = new ublox.Core.Data.UnsignedDistance(vAcc / 1000),
                Velocity = new ublox.Core.Data.Velocity3(velN / 1000, velE / 1000, velD / 1000),
                GroundSpeed = new ublox.Core.Data.SignedVelocity(gSpeed),
                HeadingOfMotion = headMot,
                SpeedAccuracyEstimate = new ublox.Core.Data.UnsignedVelocity(sAcc / 1000),
                HeadingAccuracyEstimate = headaAcc,

                PositionDilutionOfPrecision = pDOP,
                HeadingOfVehicle = headVeh,

            };
        }



        public object ToKnownMessage()
        {
            switch (Id)
            {
                case MessageId.NAV_PVT:
                    return ToNavPvt();
                default:
                    return null;
            }
        }
    }
}
