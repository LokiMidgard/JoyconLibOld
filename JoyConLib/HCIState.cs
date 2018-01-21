namespace JoyCon
{
    public enum HCIState : System.Byte
    {
        Disconnect = 0x00,
        RebootAndReconect = 0x01,
        RebootAndPair = 0x02,
        RebootAndHome = 0x04,
    }
}