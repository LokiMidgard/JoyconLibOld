using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace JoyCon
{
    static class Debug
    {
        internal static void Log(object v)
        {
            System.Diagnostics.Debug.WriteLine(v?.ToString()?? "[NULL]");
        }
    }
}
