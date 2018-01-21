using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace JoyCon
{
static    class Quartainion
    {

        /// <summary>
        /// Evaluates a rotation needed to be applied to an object positioned at sourcePoint to face destPoint
        /// </summary>
        /// <param name="sourcePoint">Coordinates of source point</param>
        /// <param name="destPoint">Coordinates of destionation point</param>
        /// <returns></returns>
        public static Quaternion LookAt(Vector3 sourcePoint, Vector3 destPoint)
        {
            var forwardVector = Vector3.Normalize(destPoint - sourcePoint);

            float dot = Vector3.Dot(Vector3.UnitX, forwardVector);

            if (Math.Abs(dot - (-1.0f)) < 0.000001f)
            {
                return new Quaternion(Vector3.UnitY.X, Vector3.UnitY.Y, Vector3.UnitY.Z, 3.1415926535897932f);
            }
            if (Math.Abs(dot - (1.0f)) < 0.000001f)
            {
                return Quaternion.Identity;
            }

            float rotAngle = (float)Math.Acos(dot);
            var rotAxis = Vector3.Cross(Vector3.UnitX, forwardVector);
            rotAxis = Vector3.Normalize(rotAxis);
            return CreateFromAxisAngle(rotAxis, rotAngle);
        }

        // just in case you need that function also
        public static Quaternion CreateFromAxisAngle(Vector3 axis, float angle)
        {
            float halfAngle = angle * .5f;
            float s = (float)System.Math.Sin(halfAngle);
            var q = new Quaternion
            {
                X = axis.X * s,
                Y = axis.Y * s,
                Z = axis.Z * s,
                W = (float)System.Math.Cos(halfAngle)
            };
            return q;
        }
    }
}
