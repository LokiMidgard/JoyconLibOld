using System.Runtime.InteropServices;
using System.Collections;
using System;
using System.Text;


public class HIDapi {
#if x64
    private const string path = @"win64\hidapi.dll";
#else
    private const string path = @"win32\hidapi";
    private const CallingConvention callConvention = CallingConvention.Cdecl;
#endif
    [DllImport(path, CallingConvention = callConvention)]
    public static extern int hid_init();

    [DllImport(path, CallingConvention = callConvention)]
    public static extern int hid_exit();

    [DllImport(path, CallingConvention = callConvention)]
    public static extern IntPtr hid_error(IntPtr device);

    [DllImport(path, CallingConvention = callConvention)]
    public static extern IntPtr hid_enumerate(ushort vendor_id, ushort product_id);

    [DllImport(path, CallingConvention = callConvention)]
    public static extern void hid_free_enumeration(IntPtr devs);

    [DllImport(path, CallingConvention = callConvention)]
    public static extern int hid_get_feature_report(IntPtr device, byte[] data, UIntPtr length);

    [DllImport(path, CallingConvention = callConvention)]
    public static extern int hid_get_indexed_string(IntPtr device, int string_index, StringBuilder str, UIntPtr maxlen);

    [DllImport(path, CallingConvention = callConvention)]
    public static extern int hid_get_manufacturer_string(IntPtr device, StringBuilder str, UIntPtr maxlen);

    [DllImport(path, CallingConvention = callConvention)]
    public static extern int hid_get_product_string(IntPtr device, StringBuilder str, UIntPtr maxlen);

    [DllImport(path, CallingConvention = callConvention)]
    public static extern int hid_get_serial_number_string(IntPtr device, StringBuilder str, UIntPtr maxlen);

    [DllImport(path, CallingConvention = callConvention)]
    public static extern IntPtr hid_open(ushort vendor_id, ushort product_id, string serial_number);

    [DllImport(path, CallingConvention = callConvention)]
    public static extern void hid_close(IntPtr device);

    [DllImport(path, CallingConvention = callConvention)]
    public static extern IntPtr hid_open_path(string path);

    [DllImport(path, CallingConvention = callConvention)]
    public static extern int hid_read(IntPtr device, byte[] data, UIntPtr length);

    [DllImport(path, CallingConvention = callConvention)]
    public static extern int hid_read_timeout(IntPtr dev, byte[] data, UIntPtr length, int milliseconds);

    [DllImport(path, CallingConvention = callConvention)]
    public static extern int hid_send_feature_report(IntPtr device, byte[] data, UIntPtr length);

    [DllImport(path, CallingConvention = callConvention)]
    public static extern int hid_set_nonblocking(IntPtr device, int nonblock);

    [DllImport(path, CallingConvention = callConvention)]
    public static extern int hid_write(IntPtr device, byte[] data, UIntPtr length);
}

struct hid_device_info {
    public string path;
    public ushort vendor_id;
    public ushort product_id;
    public string serial_number;
    public ushort release_number;
    public string manufacturer_string;
    public string product_string;
    public ushort usage_page;
    public ushort usage;
    public int interface_number;
    public IntPtr next;
}
