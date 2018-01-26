

using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System;

using System.Threading;
using System.Numerics;
using MathFloat;
using System.Collections.Concurrent;

namespace JoyCon
{

    public class Joycon
    {
        public enum DebugType : int
        {
            NONE,
            ALL,
            COMMS,
            THREADING,
            IMU,
            RUMBLE,
        };
        public DebugType debug_type = DebugType.IMU;
        public bool isLeft;
        public readonly string path;
        private readonly JoyconManager parentManager;

        public enum state_ : uint
        {
            NOT_ATTACHED,
            DROPPED,
            NO_JOYCONS,
            ATTACHED,
            INPUT_MODE_0x30,
            IMU_DATA_OK,
        };
        public state_ state;
        public enum Button : int
        {
            DPAD_DOWN = 0,
            DPAD_RIGHT = 1,
            DPAD_LEFT = 2,
            DPAD_UP = 3,
            SL = 4,
            SR = 5,
            MINUS = 6,
            HOME = 7,
            PLUS = 8,
            CAPTURE = 9,
            STICK = 10,
            SHOULDER_1 = 11,
            SHOULDER_2 = 12
        };
        private bool[] buttons_down = new bool[13];
        private bool[] buttons_up = new bool[13];
        private bool[] buttons = new bool[13];
        private bool[] down_ = new bool[13];

        private float[] stick = { 0, 0 };

        private
        IntPtr handle;

        byte[] default_buf = { 0x0, 0x1, 0x40, 0x40, 0x0, 0x1, 0x40, 0x40 };

        private byte[] stick_raw = { 0, 0, 0 };
        private UInt16[] stick_cal = { 0, 0, 0, 0, 0, 0 };
        private UInt16 deadzone;
        private UInt16[] stick_precal = { 0, 0 };

        private bool stop_polling = false;
        private int timestamp;
        private bool first_imu_packet = true;
        private bool imu_enabled = false;
        private Int16[] acc_r = { 0, 0, 0 };
        private Vector3 acc_g;

        private Int16[] gyr_r = { 0, 0, 0 };
        private Int16[] gyr_neutral = { 0, 0, 0 };
        private Vector3 gyr_g;

        private bool do_localize;
        private float filterweight;
        private const uint report_len = 49;

        private static class Subcommands
        {

            /// <summary>
            /// Bluetooth manual pairing (https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/blob/master/bluetooth_hid_subcommands_notes.md#subcommand-0x01-bluetooth-manual-pairing)
            /// </summary>
            public const byte BLUETOOTH_MANUAL_PAIRING = 0x01;

            /// <summary>
            /// Set input report mode (https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/blob/master/bluetooth_hid_subcommands_notes.md#subcommand-0x03-set-input-report-mode)
            /// </summary>
            public const byte SET_INPUT_MODE = 0x03;


            /// <summary>
            /// SPI flash read (https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/blob/master/bluetooth_hid_subcommands_notes.md#subcommand-0x10-spi-flash-read)
            /// </summary>
            public const byte SPI_FLASH_READ = 0x10;

            /// <summary>
            /// First argument byte is a bitfield:
            ///     aaaa bbbb
            ///          3210 - keep player light on (player numbers)
            ///     3210      - flash player light (player numbers)
            ///  On overrides flashing.When on USB, flashing bits work like always on bits.
            /// </summary>
            public const byte SET_PLAYER_LED = 0x30;

            /// <summary>
            /// One argument of x00 Disable or x01 Enable.
            /// </summary>
            public const byte ENABLE_IMU = 0x40;

            /// <summary>
            /// One argument of x00 Disable or x01 Enable.
            /// </summary>
            public const byte ENABLE_VIBRATION = 0x48;

            /// <summary>
            /// Causes the controller to change power state.
            /// </summary>
            internal static byte SET_HCI_STATE = 0x06;
        }




        /// <summary>
        /// Capsuls the Inputmodes for the sub command set in put mode 0x03
        /// </summary>
        private enum Inputmodes : byte
        {
            /// <summary>
            /// Simple HID mode. Pushes updates with every button press
            /// </summary>
            SIMPLE_HID = 0x3f,

            /// <summary>
            /// Standard full mode. Pushes current state @60Hz
            /// </summary>
            FULL_MODE = 0x30,

            /// <summary>
            /// NFC/IR mode. Pushes large packets @60Hz
            /// </summary>
            NFC_IR = 0x31,
        }

        private struct Report
        {
            byte[] r;
            System.DateTime t;
            public Report(byte[] report, System.DateTime time)
            {
                this.r = report;
                this.t = time;
            }
            public System.DateTime GetTime()
            {
                return this.t;
            }
            public void CopyBuffer(byte[] b)
            {
                for (int i = 0; i < report_len; ++i)
                {
                    b[i] = this.r[i];
                }
            }
        };
        private struct Rumble
        {
            private float h_f, amp, l_f;
            public float t;
            public bool timed_rumble;

            public void set_vals(float low_freq, float high_freq, float amplitude, int time = 0)
            {
                this.h_f = high_freq;
                this.amp = amplitude;
                this.l_f = low_freq;
                this.timed_rumble = false;
                this.t = 0;
                if (time != 0)
                {
                    this.t = time / 1000f;
                    this.timed_rumble = true;
                }
            }
            public Rumble(float low_freq, float high_freq, float amplitude, int time = 0)
            {
                this.h_f = high_freq;
                this.amp = amplitude;
                this.l_f = low_freq;
                this.timed_rumble = false;
                this.t = 0;
                if (time != 0)
                {
                    this.t = time / 1000f;
                    this.timed_rumble = true;
                }
            }

            private float clamp(float x, float min, float max)
            {
                if (x < min) return min;
                if (x > max) return max;
                return x;
            }

            public byte[] GetData()
            {

                var rumble_data = new byte[8];
                this.l_f = clamp(this.l_f, 40.875885f, 626.286133f);
                this.amp = clamp(this.amp, 0.0f, 1.0f);
                this.h_f = clamp(this.h_f, 81.75177f, 1252.572266f);
                var hf = (UInt16)((MathF.Round(32f * MathF.Log(this.h_f * 0.1f, 2)) - 0x60) * 4);
                byte lf = (byte)(MathF.Round(32f * MathF.Log(this.l_f * 0.1f, 2)) - 0x40);
                byte hf_amp;
                if (this.amp == 0) hf_amp = 0;
                else if (this.amp < 0.117) hf_amp = (byte)(((MathF.Log(this.amp * 1000, 2) * 32) - 0x60) / (5 - MathF.Pow(this.amp, 2)) - 1);
                else if (this.amp < 0.23) hf_amp = (byte)(((MathF.Log(this.amp * 1000, 2) * 32) - 0x60) - 0x5c);
                else hf_amp = (byte)((((MathF.Log(this.amp * 1000, 2) * 32) - 0x60) * 2) - 0xf6);

                var lf_amp = (UInt16)(MathF.Round(hf_amp) * .5);
                byte parity = (byte)(lf_amp % 2);
                if (parity > 0)
                {
                    --lf_amp;
                }

                lf_amp = (UInt16)(lf_amp >> 1);
                lf_amp += 0x40;
                if (parity > 0) lf_amp |= 0x8000;
                rumble_data = new byte[8];
                rumble_data[0] = (byte)(hf & 0xff);
                rumble_data[1] = (byte)((hf >> 8) & 0xff);
                rumble_data[2] = lf;
                rumble_data[1] += hf_amp;
                rumble_data[2] += (byte)((lf_amp >> 8) & 0xff);
                rumble_data[3] += (byte)(lf_amp & 0xff);
                for (int i = 0; i < 4; ++i)
                {
                    rumble_data[4 + i] = rumble_data[i];
                }
                //Debug.Log(string.Format("Encoded hex freq: {0:X2}", encoded_hex_freq));
                //Debug.Log(string.Format("lf_amp: {0:X4}", lf_amp));
                //Debug.Log(string.Format("hf_amp: {0:X2}", hf_amp));
                //Debug.Log(string.Format("l_f: {0:F}", l_f));
                //Debug.Log(string.Format("hf: {0:X4}", hf));
                //Debug.Log(string.Format("lf: {0:X2}", lf));
                return rumble_data;
            }
        }
        private readonly ConcurrentQueue<Report> reports = new ConcurrentQueue<Report>();
        private Rumble rumble_obj;

        private byte global_count = 0;
        private string debug_str;

        internal Joycon(IntPtr handle_, bool imu, bool localize, float alpha, bool left, string path, JoyconManager parentManager)
        {
            this.handle = handle_;
            this.imu_enabled = imu;
            this.do_localize = localize;
            this.rumble_obj = new Rumble(160, 320, 0);
            this.filterweight = alpha;
            this.isLeft = left;
            this.path = path;
            this.parentManager = parentManager;
        }

        [System.Diagnostics.Conditional("DEBUG")]
        private void DebugPrint(string s, DebugType d)
        {
            if (this.debug_type == DebugType.NONE) return;
            if (d == DebugType.ALL || d == this.debug_type || this.debug_type == DebugType.ALL)
            {
                Debug.Log(s);
            }
        }

        public bool GetButtonDown(Button b)
        {
            return this.buttons_down[(int)b];
        }
        public bool GetButton(Button b)
        {
            return this.buttons[(int)b];
        }
        public bool GetButtonUp(Button b)
        {
            return this.buttons_up[(int)b];
        }
        public float[] GetStick()
        {
            return this.stick;
        }
        public Vector3 GetGyro()
        {
            return this.gyr_g;
        }
        public Vector3 GetAccel()
        {
            return this.acc_g;
        }
        public Quaternion GetVector()
        {
            return Quartainion.LookAt(new Vector3(this.j_b.X, this.i_b.X, this.k_b.X), -(new Vector3(this.j_b.Z, this.i_b.Z, this.k_b.Z)));
            //return Quaternion.LookRotation(new Vector3(j_b.X, i_b.X, k_b.X), -(new Vector3(j_b.Z, i_b.Z, k_b.Z)));
        }

        /// <summary>
        /// Initilize the JoyCon
        /// </summary>
        /// <param name="leds_">The initial LED configuration</param>
        /// <returns></returns>
        internal int Attach(int playerNumber= 0x0)
        {
            this.state = state_.ATTACHED;
            byte[] a = { 0x0 };

            // Input report mode
            SetInputMode(Inputmodes.SIMPLE_HID);
            a[0] = 0x1;
            dump_calibration_data();
            // Connect
            byte leds_ = 0x0;
            //leds_ |= (byte)((0x1 << playerNumber) & 0b1111);
            leds_ |= (byte)(playerNumber+1);

            a[0] = leds_;
            Subcommand(Subcommands.SET_PLAYER_LED, a);
            Subcommand(Subcommands.ENABLE_IMU, new byte[] { (this.imu_enabled ? (byte)0x1 : (byte)0x0) }, true);
            SetInputMode(Inputmodes.FULL_MODE);
            Subcommand(Subcommands.ENABLE_VIBRATION, new byte[] { 0x1 }, true);

            Begin();

            //SetRumble(160, 320, 0.2f, 200);

            DebugPrint("Done with init.", DebugType.COMMS);
            return 0;
        }

        private void SetInputMode(Inputmodes inputmode)
        {
            Subcommand(Subcommands.SET_INPUT_MODE, new byte[] { (byte)inputmode }, false);
        }

        public void SetHCIState(HCIState state)
        {
            Subcommand(Subcommands.SET_HCI_STATE, new byte[] { (byte)state }, false);

        }


        public void SetFilterCoeff(float a)
        {
            this.filterweight = a;
        }

        public void Detach()
        {
            this.stop_polling = true;
            PrintArray(this.max, format: "Max {0:S}", d: DebugType.IMU);
            PrintArray(this.sum, format: "Sum {0:S}", d: DebugType.IMU);
            if (this.state > state_.NO_JOYCONS)
            {
                SetHCIState(HCIState.Disconnect);
            }
            if (this.state > state_.DROPPED)
            {
                HIDapi.hid_close(this.handle);
            }
            this.state = state_.NOT_ATTACHED;
            this.parentManager.RemoveJoyCon(this);

        }
        private byte ts_en;
        private byte ts_de;
        private System.DateTime ts_prev;

        private int ReceiveRaw()
        {
            if (this.handle == IntPtr.Zero) return -2;
            HIDapi.hid_set_nonblocking(this.handle, 0);
            var raw_buf = new byte[report_len];
            int ret = HIDapi.hid_read(this.handle, raw_buf, new UIntPtr(report_len));
            if (ret > 0)
            {
                this.reports.Enqueue(new Report(raw_buf, System.DateTime.Now));

                if (this.ts_en == raw_buf[1])
                {
                    DebugPrint(string.Format("Duplicate timestamp enqueued. TS: {0:X2}", this.ts_en), DebugType.THREADING);
                }
                this.ts_en = raw_buf[1];
                DebugPrint(string.Format("Enqueue. Bytes read: {0:D}. Timestamp: {1:X2}", ret, raw_buf[1]), DebugType.THREADING);
            }
            return ret;
        }

        private Thread PollThreadObj;
        private void Poll()
        {
            int attempts = 0;
            var time= DateTime.Now;
            DateTime oldTime;
            while (!this.stop_polling & this.state > state_.NO_JOYCONS)
            {
                oldTime = time;
                time = DateTime.Now;
                SendRumble(this.rumble_obj.GetData());
                int a = ReceiveRaw();

                if (a > 0)
                {
                    this.state = state_.IMU_DATA_OK;
                    attempts = 0;
                }
                else if (attempts > 1000)
                {
                    this.state = state_.DROPPED;
                    DebugPrint("Connection lost. Is the Joy-Con connected?", DebugType.ALL);
                    break;
                }
                else
                {
                    DebugPrint("Pause 5ms", DebugType.THREADING);
                    Thread.Sleep((Int32)5);
                }
                ++attempts;
                if (this.rumble_obj.timed_rumble)
                {
                    if (this.rumble_obj.t < 0)
                    {
                        //throw new Exception();
                        this.rumble_obj.set_vals(160, 320, 0, 0);
                    }
                    else
                    {
                        var delta = time - oldTime;
                        this.rumble_obj.t -= (float)delta.TotalMilliseconds;
                    }
                }
            }
            DebugPrint("End poll loop.", DebugType.THREADING);
        }
        float[] max = { 0, 0, 0 };
        float[] sum = { 0, 0, 0 };
        public void Update(TimeSpan delta)
        {
            if (this.state > state_.NO_JOYCONS)
            {
                var report_buf = new byte[report_len];
                while (this.reports.TryDequeue(out var rep))
                {
                    rep.CopyBuffer(report_buf);

                    if (this.imu_enabled)
                    {
                        if (this.do_localize)
                        {
                            ProcessIMU(report_buf);
                        }
                        else
                        {
                            ExtractIMUValues(report_buf, 0);
                        }
                    }
                    if (this.ts_de == report_buf[1])
                    {
                        DebugPrint(string.Format("Duplicate timestamp dequeued. TS: {0:X2}", this.ts_de), DebugType.THREADING);
                    }
                    this.ts_de = report_buf[1];
                    //DebugPrint($"Dequeue. Queue length: {this.reports.Count:d}. Packet ID: {report_buf[0]:X2}. Timestamp: {report_buf[1]:X2}. Lag to dequeue: {DateTime.Now.Subtract(rep.GetTime()):s}. Lag between packets (expect 15ms): {rep.GetTime().Subtract(this.ts_prev):s}", DebugType.THREADING);
                    this.ts_prev = rep.GetTime();
                }
                ProcessButtonsAndStick(report_buf);
                
            }
        }
        private int ProcessButtonsAndStick(byte[] report_buf)
        {
            if (report_buf[0] == 0x00) return -1;

            this.stick_raw[0] = report_buf[6 + (this.isLeft ? 0 : 3)];
            this.stick_raw[1] = report_buf[7 + (this.isLeft ? 0 : 3)];
            this.stick_raw[2] = report_buf[8 + (this.isLeft ? 0 : 3)];

            this.stick_precal[0] = (UInt16)(this.stick_raw[0] | ((this.stick_raw[1] & 0xf) << 8));
            this.stick_precal[1] = (UInt16)((this.stick_raw[1] >> 4) | (this.stick_raw[2] << 4));
            this.stick = CenterSticks(this.stick_precal);
            lock (this.buttons)
            {
                lock (this.down_)
                {
                    for (int i = 0; i < this.buttons.Length; ++i)
                    {
                        this.down_[i] = this.buttons[i];
                    }
                }
                this.buttons[(int)Button.DPAD_DOWN] = (report_buf[3 + (this.isLeft ? 2 : 0)] & (this.isLeft ? 0x01 : 0x04)) != 0;
                this.buttons[(int)Button.DPAD_RIGHT] = (report_buf[3 + (this.isLeft ? 2 : 0)] & (this.isLeft ? 0x04 : 0x08)) != 0;
                this.buttons[(int)Button.DPAD_UP] = (report_buf[3 + (this.isLeft ? 2 : 0)] & (this.isLeft ? 0x02 : 0x02)) != 0;
                this.buttons[(int)Button.DPAD_LEFT] = (report_buf[3 + (this.isLeft ? 2 : 0)] & (this.isLeft ? 0x08 : 0x01)) != 0;
                this.buttons[(int)Button.HOME] = ((report_buf[4] & 0x10) != 0);
                this.buttons[(int)Button.MINUS] = ((report_buf[4] & 0x01) != 0);
                this.buttons[(int)Button.PLUS] = ((report_buf[4] & 0x02) != 0);
                this.buttons[(int)Button.STICK] = ((report_buf[4] & (this.isLeft ? 0x08 : 0x04)) != 0);
                this.buttons[(int)Button.SHOULDER_1] = (report_buf[3 + (this.isLeft ? 2 : 0)] & 0x40) != 0;
                this.buttons[(int)Button.SHOULDER_2] = (report_buf[3 + (this.isLeft ? 2 : 0)] & 0x80) != 0;
                this.buttons[(int)Button.SR] = (report_buf[3 + (this.isLeft ? 2 : 0)] & 0x10) != 0;
                this.buttons[(int)Button.SL] = (report_buf[3 + (this.isLeft ? 2 : 0)] & 0x20) != 0;
                lock (this.buttons_up)
                {
                    lock (this.buttons_down)
                    {
                        for (int i = 0; i < this.buttons.Length; ++i)
                        {
                            this.buttons_up[i] = (this.down_[i] & !this.buttons[i]);
                            this.buttons_down[i] = (!this.down_[i] & this.buttons[i]);
                        }
                    }
                }
            }
            return 0;
        }
        private void ExtractIMUValues(byte[] report_buf, int n = 0)
        {
            this.gyr_r[0] = (Int16)(report_buf[19 + n * 12] | ((report_buf[20 + n * 12] << 8) & 0xff00));
            this.gyr_r[1] = (Int16)(report_buf[21 + n * 12] | ((report_buf[22 + n * 12] << 8) & 0xff00));
            this.gyr_r[2] = (Int16)(report_buf[23 + n * 12] | ((report_buf[24 + n * 12] << 8) & 0xff00));
            this.acc_r[0] = (Int16)(report_buf[13 + n * 12] | ((report_buf[14 + n * 12] << 8) & 0xff00));
            this.acc_r[1] = (Int16)(report_buf[15 + n * 12] | ((report_buf[16 + n * 12] << 8) & 0xff00));
            this.acc_r[2] = (Int16)(report_buf[17 + n * 12] | ((report_buf[18 + n * 12] << 8) & 0xff00));
            for (int i = 0; i < 3; ++i)
            {

                Do(ref this.acc_g, i) = this.acc_r[i] * 0.00025f;
                Do(ref this.gyr_g, i) = (this.gyr_r[i] - this.gyr_neutral[i]) * 0.00122187695f;
                if (Math.Abs(Do(ref this.acc_g, i)) > Math.Abs(this.max[i]))
                    this.max[i] = Do(ref this.acc_g, i);
            }
            ref float Do(ref Vector3 v, int index)
            {
                switch (index)
                {
                    case 0:
                        return ref v.X;
                    case 1:
                        return ref v.X;
                    case 2:
                        return ref v.X;
                    default:
                        throw new ArgumentOutOfRangeException(nameof(index), $"Value must between 0 and 2. Was {index}");
                }
            }
        }

        private float err;
        public Vector3 i_b, j_b, k_b, k_acc;
        private Vector3 d_theta;
        private Vector3 i_b_;
        private Vector3 w_a, w_g;
        private Quaternion vec;

        private int ProcessIMU(byte[] report_buf)
        {

            // Direction Cosine Matrix method
            // http://www.starlino.com/dcm_tutorial.html

            if (!this.imu_enabled | this.state < state_.IMU_DATA_OK)
                return -1;

            if (report_buf[0] != 0x30) return -1; // no gyro data

            // read raw IMU values
            int dt = (report_buf[1] - this.timestamp);
            if (report_buf[1] < this.timestamp) dt += 0x100;

            for (int n = 0; n < 3; ++n)
            {
                ExtractIMUValues(report_buf, n);

                float dt_sec = 0.005f * dt;
                this.sum[0] += this.gyr_g.X * dt_sec;
                this.sum[1] += this.gyr_g.Y * dt_sec;
                this.sum[2] += this.gyr_g.Z * dt_sec;

                if (this.isLeft)
                {
                    this.gyr_g.Y *= -1;
                    this.gyr_g.Z *= -1;
                    this.acc_g.Y *= -1;
                    this.acc_g.Z *= -1;
                }

                if (this.first_imu_packet)
                {
                    this.i_b = new Vector3(1, 0, 0);
                    this.j_b = new Vector3(0, 1, 0);
                    this.k_b = new Vector3(0, 0, 1);
                    this.first_imu_packet = false;
                }
                else
                {
                    this.k_acc = -Vector3.Normalize(this.acc_g);
                    this.w_a = Vector3.Cross(this.k_b, this.k_acc);
                    this.w_g = -this.gyr_g * dt_sec;
                    this.d_theta = (this.filterweight * this.w_a + this.w_g) / (1f + this.filterweight);
                    this.k_b += Vector3.Cross(this.d_theta, this.k_b);
                    this.i_b += Vector3.Cross(this.d_theta, this.i_b);
                    this.j_b += Vector3.Cross(this.d_theta, this.j_b);
                    //Correction, ensure new axes are orthogonal
                    this.err = Vector3.Dot(this.i_b, this.j_b) * 0.5f;
                    this.i_b_ = Vector3.Normalize(this.i_b - this.err * this.j_b);
                    this.j_b = Vector3.Normalize(this.j_b - this.err * this.i_b);
                    this.i_b = this.i_b_;
                    this.k_b = Vector3.Cross(this.i_b, this.j_b);
                }

                dt = 1;
            }
            this.timestamp = report_buf[1] + 2;
            return 0;
        }
        private void Begin()
        {
            if (this.PollThreadObj == null)
            {
                this.PollThreadObj = new Thread(new ThreadStart(this.Poll))
                {
                    IsBackground = true
                };
                this.PollThreadObj.Start();
            }
        }
        public void Recenter()
        {
            this.first_imu_packet = true;
        }
        private float[] CenterSticks(UInt16[] vals)
        {

            float[] s = { 0, 0 };
            for (uint i = 0; i < 2; ++i)
            {
                float diff = vals[i] - this.stick_cal[2 + i];
                if (Math.Abs(diff) < this.deadzone) vals[i] = 0;
                else if (diff > 0) // if axis is above center
                {
                    s[i] = diff / this.stick_cal[i];
                }
                else
                {
                    s[i] = diff / this.stick_cal[4 + i];
                }
            }
            return s;
        }
        public void SetRumble(float low_freq, float high_freq, float amp, int time = 0)
        {
            //if (this.state <= Joycon.state_.ATTACHED)
            //    return;
            if (this.rumble_obj.timed_rumble == false || this.rumble_obj.t < 0)
            {
                this.rumble_obj = new Rumble(low_freq, high_freq, amp, time);
            }
        }
        private void SendRumble(byte[] buf)
        {
            var buf_ = new byte[report_len];
            buf_[0] = 0x10;
            buf_[1] = this.global_count;
            if (this.global_count == 0xf)
                this.global_count = 0;
            else
                ++this.global_count;
            Array.Copy(buf, 0, buf_, 2, 8);
            PrintArray(buf_, DebugType.RUMBLE, format: "Rumble data sent: {0:S}");
            HIDapi.hid_write(this.handle, buf_, new UIntPtr(report_len));
        }
        private byte[] Subcommand(byte sc, byte[] buf, bool print = true)
        {
            uint len = (uint)buf.Length;
            var buf_ = new byte[report_len];
            var response = new byte[report_len];
            Array.Copy(this.default_buf, 0, buf_, 2, 8);
            Array.Copy(buf, 0, buf_, 11, len);
            buf_[10] = sc;
            buf_[1] = this.global_count;
            buf_[0] = 0x1;
            if (this.global_count == 0xf)
                this.global_count = 0;
            else
                ++this.global_count;
            if (print)
            {
                PrintArray(buf_, DebugType.COMMS, len, 11, "Subcommand 0x" + string.Format("{0:X2}", sc) + " sent. Data: 0x{0:S}");
            };
            HIDapi.hid_write(this.handle, buf_, new UIntPtr(len + 11));
            int res = HIDapi.hid_read_timeout(this.handle, response, new UIntPtr(report_len), 50);
            if (res < 1)
                DebugPrint("No response.", DebugType.COMMS);
            else if (print)
            {
                PrintArray(response, DebugType.COMMS, report_len - 1, 1, "Response ID 0x" + string.Format("{0:X2}", response[0]) + ". Data: 0x{0:S}");
            }
            return response;
        }
        private void dump_calibration_data()
        {
            var buf_ = ReadSPI(0x80, (this.isLeft ? (byte)0x12 : (byte)0x1d), 9); // get user calibration data if possible
            bool found = false;
            for (int i = 0; i < 9; ++i)
            {
                if (buf_[i] != 0xff)
                {
                    Debug.Log("Using user stick calibration data.");
                    found = true;
                    break;
                }
            }
            if (!found)
            {
                Debug.Log("Using factory stick calibration data.");
                buf_ = ReadSPI(0x60, (this.isLeft ? (byte)0x3d : (byte)0x46), 9); // get user calibration data if possible
            }
            this.stick_cal[this.isLeft ? 0 : 2] = (UInt16)((buf_[1] << 8) & 0xF00 | buf_[0]); // X Axis Max above center
            this.stick_cal[this.isLeft ? 1 : 3] = (UInt16)((buf_[2] << 4) | (buf_[1] >> 4));  // Y Axis Max above center
            this.stick_cal[this.isLeft ? 2 : 4] = (UInt16)((buf_[4] << 8) & 0xF00 | buf_[3]); // X Axis Center
            this.stick_cal[this.isLeft ? 3 : 5] = (UInt16)((buf_[5] << 4) | (buf_[4] >> 4));  // Y Axis Center
            this.stick_cal[this.isLeft ? 4 : 0] = (UInt16)((buf_[7] << 8) & 0xF00 | buf_[6]); // X Axis Min below center
            this.stick_cal[this.isLeft ? 5 : 1] = (UInt16)((buf_[8] << 4) | (buf_[7] >> 4));  // Y Axis Min below center

            PrintArray(this.stick_cal, len: 6, start: 0, format: "Stick calibration data: {0:S}");

            buf_ = ReadSPI(0x60, (this.isLeft ? (byte)0x86 : (byte)0x98), 16);
            this.deadzone = (UInt16)((buf_[4] << 8) & 0xF00 | buf_[3]);

            buf_ = ReadSPI(0x80, 0x34, 10);
            this.gyr_neutral[0] = (Int16)(buf_[0] | ((buf_[1] << 8) & 0xff00));
            this.gyr_neutral[1] = (Int16)(buf_[2] | ((buf_[3] << 8) & 0xff00));
            this.gyr_neutral[2] = (Int16)(buf_[4] | ((buf_[5] << 8) & 0xff00));
            PrintArray(this.gyr_neutral, len: 3, d: DebugType.IMU, format: "User gyro neutral position: {0:S}");

            // This is an extremely messy way of checking to see whether there is user stick calibration data present, but I've seen conflicting user calibration data on blank Joy-Cons. Worth another look eventually.
            if (this.gyr_neutral[0] + this.gyr_neutral[1] + this.gyr_neutral[2] == -3 || Math.Abs(this.gyr_neutral[0]) > 100 || Math.Abs(this.gyr_neutral[1]) > 100 || Math.Abs(this.gyr_neutral[2]) > 100)
            {
                buf_ = ReadSPI(0x60, 0x29, 10);
                this.gyr_neutral[0] = (Int16)(buf_[3] | ((buf_[4] << 8) & 0xff00));
                this.gyr_neutral[1] = (Int16)(buf_[5] | ((buf_[6] << 8) & 0xff00));
                this.gyr_neutral[2] = (Int16)(buf_[7] | ((buf_[8] << 8) & 0xff00));
                PrintArray(this.gyr_neutral, len: 3, d: DebugType.IMU, format: "Factory gyro neutral position: {0:S}");
            }
        }
        private byte[] ReadSPI(byte addr1, byte addr2, uint len, bool print = false)
        {
            byte[] buf = { addr2, addr1, 0x00, 0x00, (byte)len };
            var read_buf = new byte[len];
            var buf_ = new byte[len + 20];

            for (int i = 0; i < 100; ++i)
            {
                buf_ = Subcommand(Subcommands.SPI_FLASH_READ, buf, false);
                if (buf_[15] == addr2 && buf_[16] == addr1)
                {
                    break;
                }
            }
            Array.Copy(buf_, 20, read_buf, 0, len);
            if (print)
                PrintArray(read_buf, DebugType.COMMS, len);
            return read_buf;
        }
        private void PrintArray<T>(T[] arr, DebugType d = DebugType.NONE, uint len = 0, uint start = 0, string format = "{0:S}")
        {
            if (d != this.debug_type && this.debug_type != DebugType.ALL)
                return;
            if (len == 0)
                len = (uint)arr.Length;
            string tostr = "";
            for (int i = 0; i < len; ++i)
            {
                tostr += string.Format((arr[0] is byte) ? "{0:X2} " : ((arr[0] is float) ? "{0:F} " : "{0:D} "), arr[i + start]);
            }
            DebugPrint(string.Format(format, tostr), d);
        }
    }
}
