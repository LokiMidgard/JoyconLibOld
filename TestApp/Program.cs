using JoyCon;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.InteropServices;
using System.Threading.Tasks;

namespace TestApp
{
    class Program
    {
        static void Main(string[] args)
        {
            running = true;
            Console.WriteLine("Hello World!");

            var t = Init();

            Console.ReadKey(true);
            running = false;

            t.Wait();
        }

        [DllImport("irprops.cpl", SetLastError = true)]
        private static extern UInt32 BluetoothGetRadioInfo(IntPtr hRadio, ref BluetoothRadioInfo pRadioInfo);
        [DllImport("irprops.cpl", SetLastError = true)]
        static extern IntPtr BluetoothFindFirstRadio(ref Bluetooth_Find_Radio_Params pbtfrp, out IntPtr phRadio);

        [StructLayout(LayoutKind.Sequential)]
        private struct Bluetooth_Find_Radio_Params
        {
            internal UInt32 dwSize;
            internal void Initialize()
            {
                this.dwSize = (UInt32)Marshal.SizeOf(typeof(Bluetooth_Find_Radio_Params));
            }
        }

        private const int BLUETOOTH_MAX_NAME_SIZE = 248;
        [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode)]
        private struct BluetoothRadioInfo
        {
            internal UInt32 dwSize;
            internal UInt64 address;
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = BLUETOOTH_MAX_NAME_SIZE)]
            internal string szName;
            internal UInt32 ulClassOfDevice;
            internal UInt16 lmpSubversion;
            internal UInt16 manufacturer;

            internal void Initialize()
            {
                this.dwSize = (UInt32)Marshal.SizeOf(typeof(BluetoothRadioInfo));
            }
        }

        private static async Task Init()
        {

            var parameter = new Bluetooth_Find_Radio_Params();
            parameter.Initialize();
            Program.BluetoothFindFirstRadio(ref parameter, out var handle);

            var info = new BluetoothRadioInfo();
            info.Initialize();

            BluetoothGetRadioInfo(handle, ref info);

            byte[] mac;
            if (BitConverter.IsLittleEndian)
                mac = BitConverter.GetBytes(info.address).Take(6).ToArray();
            else
                mac = BitConverter.GetBytes(info.address).Reverse().Take(6).ToArray();



            //var devices = (await Windows.Devices.Enumeration.DeviceInformation.FindAllAsync()).Where(x=>x.Id.Contains("Bluetooth")).ToArray();
            using (var manager = new JoyconManager())
            {
                Start(manager);
                int i = 0;
                DateTime last = DateTime.Now;
                DateTime now = DateTime.Now;
                while (running)
                {

                    last = now;
                    now = DateTime.Now;
                    if ((i++ % 20 == 0))
                        manager.RefreshJoyConList();

                    Update(manager, now - last);
                    await Task.Delay(100);
                }
            }
        }

        //private static List<Joycon> joycons;

        // Values made available via Unity
        public static float[] stick;
        public static Vector3 gyro;
        private static Vector3 accel;
        public static Quaternion orientation;
        private static bool running;

        static void Start(JoyconManager manager)
        {
            gyro = new Vector3(0, 0, 0);
            accel = new Vector3(0, 0, 0);
            // get the public Joycon array attached to the JoyconManager in scene
            manager.RefreshJoyConList();
        }


        // Update is called once per frame
        static void Update(JoyconManager manager, TimeSpan delta)
        {
            manager.Update(delta);
            var joycons = manager.JoyCons;
            // make sure the Joycon only gets checked if attached
            foreach (var j in joycons)
            {
                // GetButtonDown checks if a button has been pressed (not held)
                if (j.GetButtonDown(Joycon.Button.SHOULDER_2))
                {
                    //Debug.Log("Shoulder button 2 pressed");
                    // GetStick returns a 2-element vector with x/y joystick components
                    //Debug.Log(string.Format("Stick x: {0:N} Stick y: {1:N}", j.GetStick()[0], j.GetStick()[1]));

                    // Joycon has no magnetometer, so it cannot accurately determine its yaw value. Joycon.Recenter allows the user to reset the yaw value.
                    j.Recenter();
                }
                // GetButtonDown checks if a button has been released
                if (j.GetButtonUp(Joycon.Button.SHOULDER_2))
                {
                    //Debug.Log("Shoulder button 2 released");
                }
                // GetButtonDown checks if a button is currently down (pressed or held)
                if (j.GetButton(Joycon.Button.SHOULDER_2))
                {
                    //Debug.Log("Shoulder button 2 held");
                }
                if (j.GetButtonDown(Joycon.Button.DPAD_DOWN))
                {
                    //Debug.Log("Rumble");

                    // Rumble for 200 milliseconds, with low frequency rumble at 160 Hz and high frequency rumble at 320 Hz. For more information check:
                    // https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/blob/master/rumble_data_table.md

                    j.SetRumble(160, 320, 0.6f, 200);

                    // The last argument (time) in SetRumble is optional. Call it with three arguments to turn it on without telling it when to turn off.
                    // (Useful for dynamically changing rumble values.)
                    // Then call SetRumble(0,0,0) when you want to turn it off.
                }
                stick = j.GetStick();

                // Gyro values: x, y, z axis values (in radians per second)

                gyro = j.GetGyro();

                // Accel values:  x, y, z axis values (in Gs)

                accel = j.GetAccel();
                //orientation = j.GetVector();
                //gameObject.transform.rotation = orientation;
            }

        }
    }
}
