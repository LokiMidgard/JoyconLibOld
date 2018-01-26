using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;

using System;
using System.Collections.ObjectModel;
using System.Linq;

namespace JoyCon
{

    public class JoyconManager : IDisposable
    {

        // Settings accessible via Unity
        public bool EnableIMU = true;
        public bool EnableLocalize = true;

        // Different operating systems either do or don't like the trailing zero
        private const ushort vendor_id = 0x57e;
        private const ushort vendor_id_ = 0x057e;
        private const ushort product_l = 0x2006;
        private const ushort product_r = 0x2007;
        private readonly ObservableCollection<Joycon> j; // Array of all connected Joy-Cons

        public ReadOnlyObservableCollection<Joycon> JoyCons { get; }


        public JoyconManager()
        {
            j = new ObservableCollection<Joycon>();
            HIDapi.hid_init();
            JoyCons = new ReadOnlyObservableCollection<Joycon>(j);
        }



        public void RefreshJoyConList()
        {

            bool isLeft = false;


            var ptr = HIDapi.hid_enumerate(vendor_id, 0x0);
            var top_ptr = ptr;

            if (ptr == IntPtr.Zero)
            {
                ptr = HIDapi.hid_enumerate(vendor_id_, 0x0);
                if (ptr == IntPtr.Zero)
                {
                    HIDapi.hid_free_enumeration(ptr);
                    Debug.Log("No Joy-Cons found!");
                }
            }
            hid_device_info enumerate;
            while (ptr != IntPtr.Zero)
            {
                enumerate = (hid_device_info)Marshal.PtrToStructure(ptr, typeof(hid_device_info));

                Debug.Log(enumerate.product_id);
                if (enumerate.product_id == product_l || enumerate.product_id == product_r)
                {
                    if (enumerate.product_id == product_l)
                    {
                        isLeft = true;
                        Debug.Log("Left Joy-Con connected.");
                    }
                    else if (enumerate.product_id == product_r)
                    {
                        isLeft = false;
                        Debug.Log("Right Joy-Con connected.");
                    }
                    else
                    {
                        Debug.Log("Non Joy-Con input device skipped.");
                    }
                    if (j.All(x => x.path != enumerate.path))
                    {
                        var handle = HIDapi.hid_open_path(enumerate.path);
                        HIDapi.hid_set_nonblocking(handle, 1);
                        j.Add(new Joycon(handle, EnableIMU, EnableLocalize & EnableIMU, 0.04f, isLeft, enumerate.path,this));
                    }

                }
                ptr = enumerate.next;
            }
            HIDapi.hid_free_enumeration(top_ptr);

            for (int i = 0; i < j.Count; ++i)
            {
                Debug.Log(i);
                var jc = j[i];
                if (jc.state == Joycon.state_.NOT_ATTACHED)
                {
                    jc.Attach(i);
                }
            }
        }

        public void Update(TimeSpan delta)
        {
            for (int i = 0; i < j.Count; ++i)
            {
                var jc = j[i];
                jc.Update(delta);
            }
        }


        #region IDisposable Support
        private bool disposedValue = false; // Dient zur Erkennung redundanter Aufrufe.


        protected virtual void Dispose(bool disposing)
        {
            if (!disposedValue)
            {
                if (disposing)
                {
                    while (j.Count>0)
                        j[0].Detach();
                }

                HIDapi.hid_exit();

                disposedValue = true;
            }
        }

        // TODO: Finalizer nur überschreiben, wenn Dispose(bool disposing) weiter oben Code für die Freigabe nicht verwalteter Ressourcen enthält.
        // ~JoyconManager() {
        //   // Ändern Sie diesen Code nicht. Fügen Sie Bereinigungscode in Dispose(bool disposing) weiter oben ein.
        //   Dispose(false);
        // }

        // Dieser Code wird hinzugefügt, um das Dispose-Muster richtig zu implementieren.
        public void Dispose()
        {
            // Ändern Sie diesen Code nicht. Fügen Sie Bereinigungscode in Dispose(bool disposing) weiter oben ein.
            Dispose(true);
            // TODO: Auskommentierung der folgenden Zeile aufheben, wenn der Finalizer weiter oben überschrieben wird.
            // GC.SuppressFinalize(this);
        }

        internal void RemoveJoyCon(Joycon joycon)
        {
            this.j.Remove(joycon);
        }
        #endregion
    }
}
