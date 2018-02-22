using System;
using System.IO.Ports;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using ublox.Core;

namespace TestClient
{
    public class UbloxSerialPortDeviceWindows : ISerialDevice
    {
        SerialPort _port;

        public UbloxSerialPortDeviceWindows(SerialPort port)
        {
            _port = port;

        }

        //public async Task<byte[]> ReadAsync(uint count, CancellationToken cancellationToken)
        //{
        //    var buff = new byte[count];
        //    var read = await _port.BaseStream.ReadAsync(buff, 0, (int)count, cancellationToken);
        //    if (count == read)
        //    {
        //        return buff;
        //    }
        //    else
        //    {
        //        var result = new byte[read];
        //        Array.Copy(buff, result, read);
        //        return result;
        //    }
        //}

        public void Write(byte[] data)
        {
            _port.Write(data, 0, data.Length);
        }

        public void Open()
        {
            _port.Open();
        }

        public Task<int> ReadAsync(byte[] buffer, int offset, int count, CancellationToken cancellationToken)
        {
            return _port.BaseStream.ReadAsync(buffer, offset, count, cancellationToken);
        }
    }

}
