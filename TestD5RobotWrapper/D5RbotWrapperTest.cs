//using D5RWrapperClassLibrary;
using Microsoft.VisualStudio.TestTools.UnitTesting.Logging;
using D5R;
using System.Runtime.InteropServices;

namespace TestD5RobotWrapper
{
    [TestClass]
    public sealed class D5RbotWrapperTest
    {
        [TestMethod]
        public void TestInit()
        {
            D5Robot robot = new();
            Assert.IsNotNull(robot);
            String natorID = "usb:id:2250716012";
            String topCameraId = "00-21-49-03-4D-95";
            String bottomCameraId = "00-21-49-03-4D-94";
            Assert.AreEqual(D5Robot._natorId, natorID);
            Assert.AreEqual(D5Robot._topCameraId, topCameraId);
            Assert.AreEqual(D5Robot._bottomCameraId, bottomCameraId);
            Assert.ThrowsException<SEHException>(() => robot.InitNator());

            try
            {
                robot.InitNator();
            }
            catch (SEHException ex)
            {
                Logger.LogMessage(ex.Message); //ex.Message;
            }
        }
    }
}
