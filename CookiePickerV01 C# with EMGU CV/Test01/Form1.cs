using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Features2D;
using Emgu.CV.Structure;
using Emgu.CV.Util;
using Emgu.CV.XFeatures2D;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

using DirectShowLib;






namespace Test01
{

    public partial class Form1 : Form
    {

        // #########################################
        //                 GLOBAL VARIABLE
        // #########################################

        public const double Xdivider = 6.442953020134228;
        public const double Ydivider = 5.71428571;

        int Xpos = 0;
        int Ypos = 0;
        int Zpos = 0;
        int OrienPos = 0;
        int GripPos = 1;

        bool runningFlag = false;

        #region Seiral Port Variable
        SerialPort Port;
        bool spFlag = false;
        string newcom, oldcom, acknowledge;
        int CommandCount = 0;
        int SendCommandCount = 0;
        int Commandmax = 0;
        bool readyFlag = false;
        #endregion

        DsDevice[] listCam = DsDevice.GetDevicesOfCat(FilterCategory.VideoInputDevice);
        private bool _captureInProgress;
        Emgu.CV.Capture _capture = null;

        Mat frame = new Mat();
        Mat renderFrame = new Mat();

        #region perspective transform

        PointF[] PointP = new PointF[4]
                {
                  new PointF(0, 0),
                  new PointF(640, 0),
                  new PointF(0, 320),
                  new PointF(640, 320)
                };

        Point[] PointCircle = new Point[4];


        Mat homography = new Mat();
        bool cropFlag = false;
        bool ExtractDataFlag = true;

        Byte mouseClickCount = 0;

        #endregion

        bool firstTimeFlag = true;
        Mat weightedThresholdFrame = new Mat();

        int Hupper = 255, Supper = 255, Vupper = 255;
        int Hlower = 0, Slower = 0, Vlower = 0;

        int[,] CommandData = new int[50, 6];
        //  index       meaning
        //  0           x
        //  1           y
        //  2           z
        //  3           Orientation
        //  4           type
        //              0 = Circle
        //              1 = Square
        //              2 = Rectangle
        //              3 = Triangle
        //  5           Gripping state   0 or 1

        Point[] GoalShape = new Point[5]
        {
            new Point(262,50),     //Circle Goal
            new Point(185,55),     //Square Goal
            new Point(105,60),     //Triangle Goal
            new Point(53,50),      //Rectangle Goal
            new Point(0,0)      //null
        };
        int[] OrienShape = new int[5]
        {
            0,
            0,
            90,
            90,
            0
        };


        // define color range               
        // 
        //  red aound 0           0-18
        //  yellow                22-56
        //  green                 74-143
        //  blue                  140-165
        //  purple                167-179
        //  pink                  213-241
        //
        int[] hL = new int[6]
        {   0,      //red                   
                    22,      //yellow
                    74,      //green
                    140,      //blue
                    163,      //purple
                    213       //pink                    
        };
        int[] hU = new int[6]
        {   18,      //red   
                    56,      //yellow
                    143,      //green
                    165,      //blue
                    179,      //purple
                    241       //pink
        };

        #region Initialize

        public Form1()
        {
            InitializeComponent();
            try
            {
                _capture = new Capture(0);
                _capture.Stop();

                _capture.SetCaptureProperty(CapProp.FrameHeight, 1080);
                _capture.SetCaptureProperty(CapProp.FrameWidth, 1920);
                _capture.Start();
                CameraCB.Text = listCam[0].Name;


                Hupper = HupperTrackbar.Value;
                HupperLabel.Text = "Hupper.Text : " + Hupper.ToString();
                Supper = SupperTrackbar.Value;
                SupperLabel.Text = "Supper.Text : " + Supper.ToString();
                Vupper = VupperTrackbar.Value;
                VupperLabel.Text = "Vupper.Text : " + Vupper.ToString();
                Hlower = HlowerTrackbar.Value;
                HlowerLabel.Text = "Hlower.Text : " + Hlower.ToString();
                Slower = SlowerTrackbar.Value;
                SlowerLabel.Text = "Slower.Text : " + Slower.ToString();
                Vlower = VlowerTrackbar.Value;
                VlowerLabel.Text = "Vlower.Text : " + Vlower.ToString();

                firstTimeFlag = true;

                //read point
                using (StreamReader sr = File.OpenText("PointP.txt"))
                {
                    string s = "";

                    for (int i = 0; (s = sr.ReadLine()) != null; i++)
                    {
                        PointP[i].X = int.Parse(s);
                        s = sr.ReadLine();
                        PointP[i].Y = int.Parse(s);
                    }

                    for (int i = 0; i <= 3; i++)
                    {
                        PointCircle[i] = new Point((int)PointP[i].X, (int)PointP[i].Y);
                    }
                }
                //Read Threshold
                using (StreamReader sr = File.OpenText("ThresholdValue.txt"))
                {


                    Hupper = int.Parse(sr.ReadLine());
                    Hlower = int.Parse(sr.ReadLine());
                    Supper = int.Parse(sr.ReadLine());
                    Slower = int.Parse(sr.ReadLine());
                    Vupper = int.Parse(sr.ReadLine());
                    Vlower = int.Parse(sr.ReadLine());



                }
            }
            catch (Exception e)
            {
                MessageBox.Show(e.Message);
                return;
            }

            Application.Idle += ProcessFrame;

        }

        #endregion

        double calcDist(double x1, double y1, double x2, double y2) // calc distance between 2 point
        {
            return Math.Sqrt(((x1 - x2) * (x1 - x2)) + ((y1 - y2) * (y1 - y2)));
        }

        double CalcOrien(double x1, double y1, double x2, double y2)  // calc zeta of line
        {
            return Math.Atan((y2 - y1) / (x2 - x1));
        }



        void eqHist(Mat src, Mat dst)
        {
            Mat[] channels = src.Split();

            CvInvoke.EqualizeHist(channels[0], channels[0]);
            CvInvoke.EqualizeHist(channels[1], channels[1]);
            CvInvoke.EqualizeHist(channels[2], channels[2]);
            using (VectorOfMat vm = new VectorOfMat(channels[0], channels[1], channels[2]))
            {
                CvInvoke.Merge(vm, dst);
            }
        }

        int getMaxIndex(double[] array)
        {
            double maxValue = array[0];
            int maxIndex = 0;
            int i = 1;
            for (; i < array.Length; i++)
            {
                if (array[i] > maxValue)
                {
                    maxValue = array[i];
                    maxIndex = i;
                }
            }
            return i;
        }


        /// <summary>
        /// ################################################################################
        /// 
        ///                                Main Program
        ///                     
        /// ################################################################################
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="arg"></param>
        #region Main Program  

        Mat weightedFrame = new Mat();

        void ProcessFrame(object sender, EventArgs e)
        {


            if (ExtractDataFlag == true)
            {
                #region draw the projected region on the image using Perspective Transform

                if (cropFlag)
                {
                    _capture.Retrieve(frame);
                    //DetectShapeGoal();
                    CvInvoke.WarpPerspective(frame, frame, homography, new Size(1920, 1080));

                    //for (int i = 0; i < 2; i++)
                    //{
                    //    Mat cameraFrame = new Mat();       
                    //    _capture.Retrieve(cameraFrame);
                    //    CvInvoke.FastNlMeansDenoisingColored(cameraFrame, frame);
                    //}

                }
                else
                {
                    _capture.Retrieve(frame);
                   // DetectShapeGoal();
                    frame.CopyTo(renderFrame);
                    CvInvoke.Line(renderFrame, PointCircle[0], PointCircle[1], new MCvScalar(0, 255, 0), 2);
                    CvInvoke.Line(renderFrame, PointCircle[1], PointCircle[3], new MCvScalar(0, 255, 0), 2);
                    CvInvoke.Line(renderFrame, PointCircle[3], PointCircle[2], new MCvScalar(0, 255, 0), 2);
                    CvInvoke.Line(renderFrame, PointCircle[2], PointCircle[0], new MCvScalar(0, 255, 0), 2);
                    CvInvoke.Circle(renderFrame, PointCircle[0], 3, new MCvScalar(0, 255, 0), 1);
                    CvInvoke.Circle(renderFrame, PointCircle[1], 3, new MCvScalar(0, 255, 0), 1);
                    CvInvoke.Circle(renderFrame, PointCircle[2], 3, new MCvScalar(0, 255, 0), 1);
                    CvInvoke.Circle(renderFrame, PointCircle[3], 3, new MCvScalar(0, 255, 0), 1);
                }


                #endregion

                //if (firstTimeFlag)
                //    weightedFrame = frame;
                //CvInvoke.AddWeighted(frame, 0.01, weightedFrame, 0.99, 1, weightedFrame);
                //ImgBox.Image = weightedFrame;

                //Change Contrast
                //frame.ConvertTo(frame, frame.Depth, 1.5, 0); 

                //EqHist
                //eqHist(frame, frame);               

                // Pick Order
                int ObjectOrder = 0;
                CommandCount = 0;

                Mat grayFrame = new Mat();
                Mat smoothedGrayFrame = new Mat();
                Mat frameHSV = new Mat();
                Mat thresholdFrame = new Mat();
                Mat thresholdShowFrame = new Mat();

                int[] ShapeCount = new int[5] { 0, 0, 0, 0, 0 };
                int[,] CommandDataTmp = new int[50, 6];

                //CvInvoke.GaussianBlur(frame, frame, new Size(9, 9), 20);
                CvInvoke.MedianBlur(frame, frame, 5);
                Emgu.CV.CvInvoke.CvtColor(frame, frameHSV, ColorConversion.Bgr2HsvFull);


                if (cropFlag) frame.CopyTo(renderFrame);

                if (FrameSelectCB.SelectedIndex == 2)
                    ImgBox.Image = frame;

                if (cropFlag)
                {

                    if (FrameSelectCB.SelectedIndex == 1)
                    {

                        ScalarArray upperall = new ScalarArray(new MCvScalar(Hupper, Supper, Vupper));
                        ScalarArray lowerall = new ScalarArray(new MCvScalar(Hlower, Slower, Vlower));

                        CvInvoke.InRange(frameHSV, lowerall, upperall, thresholdShowFrame);
                        //CvInvoke.BitwiseNot(thresholdShowFrame, thresholdShowFrame);
                        //UMat pyrDown = new UMat();
                        //CvInvoke.PyrDown(thresholdShowFrame, pyrDown);
                        //CvInvoke.PyrUp(pyrDown, thresholdShowFrame);
                        CvInvoke.Erode(thresholdShowFrame, thresholdShowFrame, CvInvoke.GetStructuringElement(ElementShape.Ellipse, new Size(10, 10), new Point(-1, -1)), new Point(-1, -1), 1, BorderType.Default, new MCvScalar(1, 1, 1));
                        CvInvoke.Dilate(thresholdShowFrame, thresholdShowFrame, CvInvoke.GetStructuringElement(ElementShape.Ellipse, new Size(20, 20), new Point(-1, -1)), new Point(-1, -1), 1, BorderType.Default, new MCvScalar(1, 1, 1));

                        //CvInvoke.Erode(thresholdShowFrame, thresholdShowFrame, CvInvoke.GetStructuringElement(ElementShape.Ellipse, new Size(9, 9), new Point(-1, -1)), new Point(-1, -1), 1, BorderType.Default, new MCvScalar(1, 1, 1));


                        ImgBox.Image = thresholdShowFrame;
                    }
                    if (FrameSelectCB.SelectedIndex == 0)
                    {
                        for (int colorSet = 0; colorSet <= 5; colorSet++)
                        {

                            #region Thresholding                              

                            ////Define range

                            //ScalarArray upper = new ScalarArray(new MCvScalar(Hupper, Supper, Vupper));
                            //ScalarArray lower = new ScalarArray(new MCvScalar(Hlower, Slower, Vlower));

                            ScalarArray lower = new ScalarArray(new MCvScalar(hL[colorSet], Slower, 0));
                            ScalarArray upper = new ScalarArray(new MCvScalar(hU[colorSet], Supper, 255));

                            // Thresholding using H in HSV
                            CvInvoke.InRange(frameHSV, lower, upper, thresholdFrame);
                            CvInvoke.BitwiseNot(thresholdFrame, thresholdFrame);

                            // reduce noise
                            //UMat pyrDown = new UMat();
                            //CvInvoke.PyrDown(thresholdFrame, pyrDown);
                            //CvInvoke.PyrUp(pyrDown, thresholdFrame);

                            CvInvoke.Erode(thresholdFrame, thresholdFrame, CvInvoke.GetStructuringElement(ElementShape.Ellipse, new Size(10, 10), new Point(-1, -1)), new Point(-1, -1), 1, BorderType.Default, new MCvScalar(1, 1, 1));
                            CvInvoke.Dilate(thresholdFrame, thresholdFrame, CvInvoke.GetStructuringElement(ElementShape.Ellipse, new Size(20, 20), new Point(-1, -1)), new Point(-1, -1), 1, BorderType.Default, new MCvScalar(1, 1, 1));


                            #endregion

                            Mat ContourFrame = thresholdFrame;

                            #region Detect Shape

                            List<Triangle2DF> triangleList = new List<Triangle2DF>();
                            List<RotatedRect> boxList = new List<RotatedRect>(); //a box is a rotated rectangle

                            Mat hierachy = new Mat();
                            VectorOfVectorOfPoint contours = new VectorOfVectorOfPoint();
                            CvInvoke.FindContours(ContourFrame, contours, hierachy, RetrType.List, ChainApproxMethod.ChainApproxSimple);


                            #region each contour Operation

                            int count = contours.Size;

                            for (int i = 0; i < count; i++)
                            {
                                int type = 4;       // 4 is mean nothing
                                double orientation = 0;
                                VectorOfPoint approxContour = new VectorOfPoint();
                                CvInvoke.ApproxPolyDP(contours[i], approxContour, CvInvoke.ArcLength(contours[i], true) * 0.04, true);

                                double a = CvInvoke.ContourArea(contours[i], false);  //  Find the area of contour                        


                                //Filter contour area > 4,000 and < 100,000   to ignore noise with small area
                                if (a > 4000 && a < 100000)
                                {

                                    #region Extract Data from contour
                                    Point[] pts = approxContour.ToArray();  // Point angle
                                    for (int j = 0; j < approxContour.Size; j++)
                                        CvInvoke.Circle(renderFrame, pts[j], 10, new MCvScalar(255, 0, 255), -1);

                                    //Calc center Point
                                    MCvMoments m = new MCvMoments();
                                    m = CvInvoke.Moments(contours[i]);
                                    double cx = m.M10 / m.M00;
                                    double cy = m.M01 / m.M00;
                                    double realCx = (1920 - cx) / Xdivider;
                                    double realCy = cy / Ydivider;


                                    //Orientation Method 1 using min area rectangle
                                    RotatedRect rRect = CvInvoke.MinAreaRect(approxContour);
                                    //CvInvoke.PutText(renderFrame, rRect.Angle.ToString(), new Point((int)cx, (int)cy + 50), FontFace.HersheyComplex, 2, new MCvScalar(255, 170, 80), 3);

                                    orientation = (int)rRect.Angle;

                                    #endregion


                                    CvInvoke.Circle(renderFrame, new Point((int)cx, (int)cy), 5, new MCvScalar(255, 255, 0), -1);

                                    #region Triangle

                                    if (approxContour.Size == 3)
                                    {
                                        CvInvoke.DrawContours(renderFrame, contours, i, new MCvScalar(0, 0, 255), 3);

                                        // หา Orientation ด้วยวิธี  เอามุม ของเส้น จากศูนย์กลาง ไปจุดที่ใกล้ที่สุด                                        

                                        #region Get Orientation
                                        double maxDist = 0;
                                        Point[] pointAtMax = new Point[2];
                                        Point[] pointHigh = new Point[1];



                                        double maxDistTmp = calcDist(pts[0].X, pts[0].Y, pts[1].X, pts[1].Y);
                                        if (maxDistTmp > maxDist)
                                        {
                                            maxDist = maxDistTmp;
                                            pointAtMax[0] = pts[0];
                                            pointAtMax[1] = pts[1];
                                            pointHigh[0] = pts[2];
                                        }

                                        maxDistTmp = calcDist(pts[1].X, pts[1].Y, pts[2].X, pts[2].Y);
                                        if (maxDistTmp > maxDist)
                                        {
                                            maxDist = maxDistTmp;
                                            pointAtMax[0] = pts[1];
                                            pointAtMax[1] = pts[2];
                                            pointHigh[0] = pts[0];
                                        }

                                        maxDistTmp = calcDist(pts[2].X, pts[2].Y, pts[0].X, pts[0].Y);
                                        if (maxDistTmp > maxDist)
                                        {
                                            maxDist = maxDistTmp;
                                            pointAtMax[0] = pts[0];
                                            pointAtMax[1] = pts[2];
                                            pointHigh[0] = pts[1];
                                        }

                                        orientation = CalcOrien(pointAtMax[0].X, pointAtMax[0].Y, pointAtMax[1].X, pointAtMax[1].Y);
                                        orientation = (orientation * (180 / Math.PI));


                                        if (((pointAtMax[0].Y + pointAtMax[1].Y) / 2) >= pointHigh[0].Y && ((pointAtMax[0].X + pointAtMax[1].X) / 2) >= pointHigh[0].X)
                                        {
                                            orientation = Math.Abs(orientation);
                                            //richTextBox1.Text = "Q1";
                                            //Q1
                                        }
                                        if (((pointAtMax[0].Y + pointAtMax[1].Y) / 2) <= pointHigh[0].Y && ((pointAtMax[0].X + pointAtMax[1].X) / 2) >= pointHigh[0].X)
                                        {
                                            orientation = 180 - orientation;
                                            //richTextBox1.Text = "Q2";
                                            //Q2
                                        }

                                        if (((pointAtMax[0].Y + pointAtMax[1].Y) / 2) < pointHigh[0].Y && ((pointAtMax[0].X + pointAtMax[1].X) / 2) < pointHigh[0].X)
                                        {
                                            orientation = Math.Abs(orientation) + 180;
                                            //richTextBox1.Text = "Q3";
                                            //Q3
                                        }
                                        if (((pointAtMax[0].Y + pointAtMax[1].Y) / 2) > pointHigh[0].Y && ((pointAtMax[0].X + pointAtMax[1].X) / 2) < pointHigh[0].X)
                                        {
                                            orientation = 360 - orientation;
                                            // richTextBox1.Text = "Q4";
                                            //Q4
                                        }
                                        #endregion

                                        type = 2;
                                        ShapeCount[2]++;
                                    }

                                    #endregion

                                    #region Rectangle 

                                    if (approxContour.Size == 4)
                                    {
                                        float ratio = rRect.Size.Width / rRect.Size.Height;
                                        double maxDist = 0;
                                        Point[] pointAtMax = new Point[2];

                                        //Recognition rectangle & square with ratio of side by side

                                        //CvInvoke.PutText(renderFrame, ratio.ToString(), new Point((int)cx - 100, (int)cy), FontFace.HersheyComplex, 3, new MCvScalar(255, 255, 0),5);
                                        // Square
                                        if (ratio > 0.6 && ratio < 1.4)
                                        {
                                            #region Get Orientation                                            

                                            //orientation = CalcOrien(pts[0].X, pts[0].Y, pts[1].X, pts[1].Y);
                                            //orientation = (Math.Abs(orientation) * (180 / Math.PI));

                                            orientation = Math.Abs(rRect.Angle);

                                            #endregion*/
                                            type = 1;
                                            ShapeCount[1]++;

                                            CvInvoke.DrawContours(renderFrame, contours, i, new MCvScalar(255, 0, 0), 3);


                                        }
                                        // Rectangle
                                        else
                                        {
                                            #region Get Orientation
                                            double maxDistTmp = calcDist(pts[0].X, pts[0].Y, pts[1].X, pts[1].Y);
                                            if (maxDistTmp > maxDist)
                                            {
                                                maxDist = maxDistTmp;
                                                pointAtMax[0] = pts[0];
                                                pointAtMax[1] = pts[1];
                                            }
                                            maxDistTmp = calcDist(pts[1].X, pts[1].Y, pts[2].X, pts[2].Y);
                                            if (maxDistTmp > maxDist)
                                            {
                                                maxDist = maxDistTmp;
                                                pointAtMax[0] = pts[1];
                                                pointAtMax[1] = pts[2];
                                            }

                                            orientation = CalcOrien(pointAtMax[0].X, pointAtMax[0].Y, pointAtMax[1].X, pointAtMax[1].Y);
                                            orientation = (orientation * (180 / Math.PI));

                                            if (orientation <= 0)
                                            {
                                                orientation = Math.Abs(orientation);
                                            }
                                            else
                                            {
                                                orientation = 180 - orientation;
                                            }

                                            //orientation = Math.Abs( rRect.Angle);

                                            #endregion
                                            type = 3;
                                            ShapeCount[3]++;
                                            CvInvoke.DrawContours(renderFrame, contours, i, new MCvScalar(255, 255, 0), 3);

                                        }

                                    }
                                    #endregion

                                    #region Circle

                                    if (approxContour.Size > 4)
                                    {
                                        type = 0;
                                        ShapeCount[0]++;
                                        orientation = 0;
                                        CvInvoke.DrawContours(renderFrame, contours, i, new MCvScalar(0, 255, 0), 3, LineType.FourConnected);
                                        CvInvoke.Circle(renderFrame, new Point((int)cx, (int)cy), 5, new MCvScalar(0, 255, 0), -1);
                                    }
                                    #endregion


                                    // More than one in Slot

                                    int ZplacePos = 320 - (100 * (ShapeCount[type] - 1));

                                    if (ZplacePos < 0) ZplacePos = 0;





                                    #region Command Builder
                                    //  index       meaning
                                    //  0           x
                                    //  1           y
                                    //  2           z
                                    //  3           Orientation
                                    //  4           type
                                    //              0 = Circle
                                    //              1 = Square
                                    //              2 = Triangle
                                    //              3 = Rectangle
                                    //  5           Gripping state   0 or 1

                                    // Print Order
                                    //  cx is X
                                    //  cy is Y  of center object

                                    //  rRect.Angle   is orientation

                                    //  approxContour.Size is tell what shape are

                                    //  ObjectCount is pick order



                                    if (orientation > 360)
                                    {
                                        orientation = orientation - 360;
                                    }
                                    if (CommandCount < 49)
                                    {
                                        //To pick
                                        CommandDataTmp[CommandCount, 0] = (int)realCx;
                                        CommandDataTmp[CommandCount, 1] = (int)realCy + 111;
                                        CommandDataTmp[CommandCount, 2] = 320;               //Z
                                        CommandDataTmp[CommandCount, 3] = (int)orientation;  // float to int beware of floating
                                        CommandDataTmp[CommandCount, 4] = type;
                                        CommandDataTmp[CommandCount, 5] = 0;            // To grip

                                        CommandCount++;

                                        //To place

                                        CommandDataTmp[CommandCount, 0] = GoalShape[type].X;
                                        CommandDataTmp[CommandCount, 1] = GoalShape[type].Y;
                                        //CommandDataTmp[CommandCount, 0] = 0;
                                        //CommandDataTmp[CommandCount, 1] = 0;
                                        CommandDataTmp[CommandCount, 2] = ZplacePos;      //Zaxis
                                        CommandDataTmp[CommandCount, 3] = OrienShape[type];
                                        CommandDataTmp[CommandCount, 4] = type;
                                        CommandDataTmp[CommandCount, 5] = 1;            // To release

                                        CommandCount++;
                                    }

                                    #endregion

                                    #region Draw MinAreaRect
                                    //PointF[] PointRrect = rRect.GetVertices();
                                    //CvInvoke.Line(renderFrame, new Point((int)PointRrect[0].X, (int)PointRrect[0].Y), new Point((int)PointRrect[1].X, (int)PointRrect[1].Y), new MCvScalar(0, 255, 0), 2);
                                    //CvInvoke.Line(renderFrame, new Point((int)PointRrect[1].X, (int)PointRrect[1].Y), new Point((int)PointRrect[2].X, (int)PointRrect[2].Y), new MCvScalar(0, 255, 0), 2);
                                    //CvInvoke.Line(renderFrame, new Point((int)PointRrect[2].X, (int)PointRrect[2].Y), new Point((int)PointRrect[3].X, (int)PointRrect[3].Y), new MCvScalar(0, 255, 0), 2);
                                    //CvInvoke.Line(renderFrame, new Point((int)PointRrect[3].X, (int)PointRrect[3].Y), new Point((int)PointRrect[0].X, (int)PointRrect[0].Y), new MCvScalar(0, 255, 0), 2);
                                    #endregion

                                    //CvInvoke.PutText(renderFrame, colorSet.ToString(), new Point((int)cx + 100, (int)cy), FontFace.HersheyComplex, 3, new MCvScalar(255, 255, 0), 3);

                                    CvInvoke.PutText(renderFrame, orientation.ToString(), new Point((int)cx, (int)cy + 50), FontFace.HersheyComplex, 1, new MCvScalar(255, 255, 255), 2);

                                    CvInvoke.PutText(renderFrame, ((int)(realCx)).ToString() + ',' + ((int)(realCy)).ToString(), new Point((int)cx, (int)cy), FontFace.HersheyComplex, 1, new MCvScalar(255, 255, 255), 2);

                                    CvInvoke.PutText(renderFrame, ObjectOrder.ToString(), new Point((int)cx, (int)cy - 40), FontFace.HersheyComplex, 2, new MCvScalar(255, 255, 255), 3);

                                    CvInvoke.PutText(renderFrame, colorSet.ToString(), new Point((int)cx, (int)cy + 100), FontFace.HersheyComplex, 1, new MCvScalar(255, 255, 255), 2);

                                    ObjectOrder++;

                                }

                            }



                            #endregion

                            firstTimeFlag = false;

                        }
                    }
                    #endregion
                }

                if (FrameSelectCB.SelectedIndex == 0)
                    ImgBox.Image = renderFrame;

                Commandmax = CommandCount;

                CommandCount = 0;
                CommandData = CommandDataTmp;

            }

        }

        #endregion


        #region Serial Port Operation Here
        //To revieve Protocol and get to rich Text

        private void UpdateUi(string y)
        {
            acknowledge = y;
            richTextBox1.AppendText("MCU : " + acknowledge + "\n");  //Add value to richTextBox
            if (acknowledge == "Z" && runningFlag == true)
            {
                readyFlag = true;

                SendCommandSet('S', 'P', CommandData, 'E');
            }

        }

        private void button2_Click(object sender, EventArgs e)
        {
            Port.Write("Z1#");

        }

        private void button1_Click(object sender, EventArgs e)
        {
            if (spFlag == false)
            {
                try
                {
                    Port = new SerialPort(ComportCombobox.Text, 9600, Parity.None, 8, StopBits.One);


                    Port.DataReceived += new SerialDataReceivedEventHandler(serialPort1_DataReceived);
                    Port.Open();
                    if (Port.IsOpen)
                    {
                        button1.BackColor = Color.LightGreen;
                        button1.Text = "Disconnect";
                        spFlag = true;
                        MachineJoggingGB.Enabled = true;

                        if (ExtractDataFlag)
                            RunBT.Enabled = true;
                    }
                }
                catch { }
            }
            else
            {
                MachineJoggingGB.Enabled = false;
                Port.Close();
                button1.BackColor = Color.WhiteSmoke;
                button1.Text = "Connect";
                spFlag = false;
            }
        }

        void SendCommandSet(char start, char command, int[,] Data, char stop)
        {

            // Keep data to Buffer


            if (readyFlag == true)
            {
                readyFlag = false;
                int i = SendCommandCount;

                XposLabel.Text = Data[i, 0].ToString();
                YposLabel.Text = Data[i, 1].ToString();
                ZposLabel.Text = Data[i, 2].ToString();

                richTextBox1.AppendText("Command : " + CommandCount.ToString() + "SP " + Data[i, 0] + " " + Data[i, 1] + " " + Data[i, 2] + " " + Data[i, 3] + " " + Data[i, 4] + " " + Data[i, 5] + "E\n");

                byte[] buffer = new byte[13];

                buffer[0] = Convert.ToByte(start);
                buffer[1] = Convert.ToByte(command);
                buffer[2] = Convert.ToByte((int)(Data[i, 0] / 256));
                buffer[3] = Convert.ToByte(Data[i, 0] % 256);
                buffer[4] = Convert.ToByte((int)Data[i, 1] / 256);
                buffer[5] = Convert.ToByte(Data[i, 1] % 256);
                buffer[6] = Convert.ToByte((int)Data[i, 2] / 256);
                buffer[7] = Convert.ToByte(Data[i, 2] % 256);
                buffer[8] = Convert.ToByte((int)Data[i, 3] / 256);
                buffer[9] = Convert.ToByte(Data[i, 3] % 256);
                buffer[10] = Convert.ToByte(Data[i, 4]);
                buffer[11] = Convert.ToByte(Data[i, 5]);
                buffer[12] = Convert.ToByte(stop);
                Port.Write(buffer, 0, 13);



                debugText.Text = "Going to " + (i + 1).ToString() + " Object";
                acknowledge = "";
                SendCommandCount++;

                if (SendCommandCount > Commandmax)
                {
                    richTextBox1.AppendText("FINISH\n");
                    runningFlag = false;
                    Port.Write("SH0000000000E");
                    richTextBox1.AppendText("Auto Home : SH0000000000E\n");
                    XposLabel.Text = "0";
                    YposLabel.Text = "0";
                    ZposLabel.Text = "0";
                    Xpos = 0;
                    Ypos = 0;
                    Zpos = 0;
                    debugText.Text = "Auto Home.";
                }


            }


            /*if (acknowledge.Contains("SCE")) { sp.Write(buffer, 0, 13); }
      if (acknowledge.Contains("SYE")) { return false; }
      if (acknowledge.Contains("SNE")) { return false; }
      return false;*/

        }

        void SendCommand(char start, char command, int X, int Y, int Z, int orien, int grip)
        {
            byte[] buffer = new byte[13];

            buffer[0] = Convert.ToByte(start);
            buffer[1] = Convert.ToByte(command);
            buffer[2] = Convert.ToByte(X / 256);
            buffer[3] = Convert.ToByte(X % 256);
            buffer[4] = Convert.ToByte(Y / 256);
            buffer[5] = Convert.ToByte(Y % 256);
            buffer[6] = Convert.ToByte(Z / 256);
            buffer[7] = Convert.ToByte(Z % 256);
            buffer[8] = Convert.ToByte(orien / 256);
            buffer[9] = Convert.ToByte(orien % 256);
            buffer[10] = Convert.ToByte(0);
            buffer[11] = Convert.ToByte(grip);
            buffer[12] = Convert.ToByte('E');
            Port.Write(buffer, 0, 13);

        }





        #endregion



        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            _capture.Stop();
            _capture.Dispose();

            if (spFlag)
                Port.Close();

            //Save Threshold
            string path = "ThresholdValue.txt";


            // Create a file to write to.
            using (StreamWriter sw = File.CreateText(path))
            {
                sw.WriteLine(Hupper.ToString());
                sw.WriteLine(Hlower.ToString());

                sw.WriteLine(Supper.ToString());
                sw.WriteLine(Slower.ToString());

                sw.WriteLine(Vupper.ToString());
                sw.WriteLine(Vlower.ToString());
            }

        }

        // Crop button
        private void BTcrop_Click(object sender, EventArgs e)
        {
            if (cropFlag == false) {
                PointF[] PointCropTo = new PointF[4]
                  {
                  new PointF(0, 0),
                  new PointF(frame.Cols, 0),
                  new PointF(0, frame.Rows),
                  new PointF(frame.Cols,frame.Rows)
                  };

                string path = "PointP.txt";


                // Create a file to write to.
                using (StreamWriter sw = File.CreateText(path))
                {
                    sw.WriteLine(PointP[0].X.ToString());
                    sw.WriteLine(PointP[0].Y.ToString());

                    sw.WriteLine(PointP[1].X.ToString());
                    sw.WriteLine(PointP[1].Y.ToString());

                    sw.WriteLine(PointP[2].X.ToString());
                    sw.WriteLine(PointP[2].Y.ToString());

                    sw.WriteLine(PointP[3].X.ToString());
                    sw.WriteLine(PointP[3].Y.ToString());
                }


                CvInvoke.FindHomography(PointP, PointCropTo, homography, HomographyMethod.Ransac);

                cropFlag = true;
                BTcrop.Text = "Reset Crop";
                debugText.Text = "Perspective Tranform Complete.";

                ButtonExtractData.Enabled = true;

                HupperTrackbar.Enabled = true;
                HlowerTrackbar.Enabled = true;
                SupperTrackbar.Enabled = true;
                SlowerTrackbar.Enabled = true;
                VupperTrackbar.Enabled = true;
                VlowerTrackbar.Enabled = true;
            }
            else
            {
                cropFlag = false;
                BTcrop.Text = "Crop";
                debugText.Text = "Reset Perspective.";
                firstTimeFlag = true;
                ButtonExtractData.Enabled = false;

                HupperTrackbar.Enabled = false;
                HlowerTrackbar.Enabled = false;
                SupperTrackbar.Enabled = false;
                SlowerTrackbar.Enabled = false;
                VupperTrackbar.Enabled = false;
                VlowerTrackbar.Enabled = false;
            }

        }


        //Click to Crop
        private void ImgBox_MouseDown(object sender, MouseEventArgs e)
        {
            if (ImgBox.Image != null)
            {
                // "X Pos: " + e.X.ToString();
                // "Y Pos: " + e.Y.ToString();

                float pointX = e.X * 2;
                float pointY = e.Y * 2;

                switch (mouseClickCount)
                {
                    case 0:
                        PointP[0] = new PointF(pointX, pointY);
                        PointCircle[0] = new Point((int)pointX, (int)pointY);
                        debugText.Text = "Point 1 :" + "X = " + PointP[0].X.ToString() + " Y = " + PointP[0].Y.ToString();
                        mouseClickCount = 1;
                        break;
                    case 1:
                        PointP[1] = new PointF(pointX, pointY);
                        PointCircle[1] = new Point((int)pointX, (int)pointY);
                        debugText.Text = "Point 2 :" + "X = " + PointP[1].X.ToString() + " Y = " + PointP[1].Y.ToString();
                        mouseClickCount = 2;
                        break;
                    case 2:
                        PointP[2] = new PointF(pointX, pointY);
                        PointCircle[2] = new Point((int)pointX, (int)pointY);
                        debugText.Text = "Point 3 :" + "X = " + PointP[2].X.ToString() + " Y = " + PointP[2].Y.ToString();
                        mouseClickCount = 3;
                        break;
                    case 3:
                        PointP[3] = new PointF(pointX, pointY);
                        PointCircle[3] = new Point((int)pointX, (int)pointY);
                        debugText.Text = "Point 4 :" + "X = " + PointP[3].X.ToString() + " Y = " + PointP[3].Y.ToString();
                        mouseClickCount = 0;
                        break;
                    default:
                        mouseClickCount = 0;
                        break;
                }


            }
        }

        // Credit just for fun
        private void linkLabel1_LinkClicked(object sender, LinkLabelLinkClickedEventArgs e)
        {
            MessageBox.Show("FIBO FRAB#1 GROUP 25");
        }


        #region Trackbar

        private void HupperTrackbar_Scroll(object sender, EventArgs e)
        {
            Hupper = HupperTrackbar.Value;
            HupperLabel.Text = "Hupper : " + Hupper.ToString();

        }

        private void SupperTrackbar_Scroll(object sender, EventArgs e)
        {
            Supper = SupperTrackbar.Value;
            SupperLabel.Text = "Supper : " + Supper.ToString();
        }

        private void VupperTrackbar_Scroll(object sender, EventArgs e)
        {
            Vupper = VupperTrackbar.Value;
            VupperLabel.Text = "Vupper : " + Vupper.ToString();
        }

        private void HlowerTrackbar_Scroll(object sender, EventArgs e)
        {
            Hlower = HlowerTrackbar.Value;
            HlowerLabel.Text = "Hlower : " + Hlower.ToString();
        }

        private void SlowerTrackbar_Scroll(object sender, EventArgs e)
        {
            Slower = SlowerTrackbar.Value;
            SlowerLabel.Text = "Slower : " + Slower.ToString();
        }

        private void VlowerTrackbar_Scroll(object sender, EventArgs e)
        {
            Vlower = VlowerTrackbar.Value;
            VlowerLabel.Text = "Vlower : " + Vlower.ToString();
        }

        #endregion

        // Extract data
        private void button8_Click(object sender, EventArgs e)
        {
            ExtractDataFlag = !ExtractDataFlag;

            if (ExtractDataFlag == false)
            {
                ButtonExtractData.Text = "Reset Data";

                if (spFlag)
                    RunBT.Enabled = true;

            }
            else
            {
                ButtonExtractData.Text = "Extract Data";
                RunBT.Enabled = false;
                SendCommandCount = 0;
            }


        }


        private void comboBox2_DropDown(object sender, EventArgs e)
        {
            ComportCombobox.Items.Clear();
            string[] ports = SerialPort.GetPortNames();
            foreach (string comport in ports)
            {
                ComportCombobox.Items.Add(comport);
            }
        }


        private void ComportCombobox_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (newcom != oldcom) { Port.Close(); }

            newcom = ComportCombobox.Text;

        }

        private void button9_Click(object sender, EventArgs e)
        {
          
            runningFlag = true;

            readyFlag = true;

            //using (var dialog = new System.Windows.Forms.SaveFileDialog())
            //{
            //    dialog.DefaultExt = "Cookie_Position.txt";
            //    dialog.Filter = "Text files (*.txt)|*.txt|All files (*.*)|*.*";
            //    DialogResult result = dialog.ShowDialog();
            //    if (result == DialogResult.OK)
            //    {
            //        string filename = dialog.FileName;

            //        using (StreamWriter outputFile = new StreamWriter(filename, true))
            //        {
            //            outputFile.WriteLine(CommandData.ToString());
            //        }


            //    }
            //}

            SendCommandSet('S', 'P', CommandData, 'E');
            // Run with ObjectData 0 to Command Count
            debugText.Text = "Running.";

        }



        private void exitToolStripMenuItem_Click(object sender, EventArgs e)
        {
            Application.Exit();
        }

        private void CameraCB_DropDown(object sender, EventArgs e)
        {
            for (int idx = 0; idx < listCam.Length; idx++)
            {
                // Do something with the device here...
                CameraCB.Items.Add(listCam[idx].Name);
            }
        }

        private void CameraCB_SelectionChangeCommitted(object sender, EventArgs e)
        {
            _capture.Stop();
            _capture.Dispose();
            _capture = new Emgu.CV.Capture(CameraCB.TabIndex);
            _capture.Stop();
            _capture.SetCaptureProperty(CapProp.FrameHeight, 1080);
            _capture.SetCaptureProperty(CapProp.FrameWidth, 1920);
            _capture.Start();
        }



        private void Form1_Load(object sender, EventArgs e)
        {
            FrameSelectCB.SelectedIndex = 0;

        }

        private void buttonHome_Click(object sender, EventArgs e)
        {
            Port.Write("SH0000000000E");
            richTextBox1.AppendText("Auto Home : SH0000000000E\n");
            XposLabel.Text = "0";
            YposLabel.Text = "0";
            ZposLabel.Text = "0";
            Xpos = 0;
            Ypos = 0;
            Zpos = 0;
            debugText.Text = "Auto Home.";
        }

        private void serialPort1_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            try
            {
                string line = Port.ReadExisting();

                Invoke((MethodInvoker)(() => UpdateUi(line)));


            }
            catch { }

        }

        private void jettananToolStripMenuItem_Click(object sender, EventArgs e)
        {

        }

        private void jadsadakornToolStripMenuItem_Click(object sender, EventArgs e)
        {

        }

        private void richTextBox1_TextChanged(object sender, EventArgs e)
        {

        }

        #region Jogging

        private void button2_Click_1(object sender, EventArgs e)
        {
            Ypos += (int)step.Value;
            if (Ypos > 300) Ypos = 300;
            if (Ypos < 0) Ypos = 0;
            SendCommand('S', 'P', Xpos, Ypos, Zpos, OrienPos, GripPos);
            YposLabel.Text = Ypos.ToString();
            richTextBox1.AppendText("Mechine Jog : SP " + Xpos.ToString() + " " + Ypos.ToString() + " " + Zpos.ToString() + " " + OrienPos.ToString() + " " + GripPos.ToString() + "E\n");
        }

        private void button5_Click(object sender, EventArgs e)
        {
            Xpos -= (int)step.Value;
            if (Xpos > 300) Xpos = 300;
            if (Xpos < 0) Xpos = 0;
            SendCommand('S', 'P', Xpos, Ypos, Zpos, OrienPos, GripPos);
            XposLabel.Text = Xpos.ToString();
            richTextBox1.AppendText("Mechine Jog : SP " + Xpos.ToString() + " " + Ypos.ToString() + " " + Zpos.ToString() + " " + OrienPos.ToString() + " " + GripPos.ToString() + "E\n");
        }
        private void button4_Click(object sender, EventArgs e)
        {
            Ypos -= (int)step.Value;
            if (Ypos > 300) Ypos = 300;
            if (Ypos < 0) Ypos = 0;
            SendCommand('S', 'P', Xpos, Ypos, Zpos, OrienPos, GripPos);
            YposLabel.Text = Ypos.ToString();
            richTextBox1.AppendText("Mechine Jog : SP " + Xpos.ToString() + " " + Ypos.ToString() + " " + Zpos.ToString() + " " + OrienPos.ToString() + " " + GripPos.ToString() + "E\n");
        }

        private void button3_Click(object sender, EventArgs e)
        {
            Xpos += (int)step.Value;
            if (Xpos > 300) Xpos = 300;
            if (Xpos < 0) Xpos = 0;
            SendCommand('S', 'P', Xpos, Ypos, Zpos, OrienPos, GripPos);
            XposLabel.Text = Xpos.ToString();

            richTextBox1.AppendText("Mechine Jog : SP " + Xpos.ToString() + " " + Ypos.ToString() + " " + Zpos.ToString() + " " + OrienPos.ToString() + " " + GripPos.ToString() + "E\n");
        }

        private void button6_Click(object sender, EventArgs e)
        {
            Zpos += (int)step.Value;
            if (Zpos > 320) Zpos = 100;
            if (Zpos < 0) Zpos = 0;
            SendCommand('S', 'P', Xpos, Ypos, Zpos, OrienPos, GripPos);
            ZposLabel.Text = Zpos.ToString();
            richTextBox1.AppendText("Mechine Jog : SP " + Xpos.ToString() + " " + Ypos.ToString() + " " + Zpos.ToString() + " " + OrienPos.ToString() + " " + GripPos.ToString() + "E\n");
        }

        private void button7_Click(object sender, EventArgs e)
        {
            Zpos -= (int)step.Value;
            if (Zpos > 320) Zpos = 100;
            if (Zpos < 0) Zpos = 0;
            SendCommand('S', 'P', Xpos, Ypos, Zpos, OrienPos, GripPos);
            ZposLabel.Text = Zpos.ToString();
            richTextBox1.AppendText("Mechine Jog : SP " + Xpos.ToString() + " " + Ypos.ToString() + " " + Zpos.ToString() + " " + OrienPos.ToString() + " " + GripPos.ToString() + "E\n");

        }
        #endregion


        //private void DetectShapeGoal()
        //{
        //    Mat GoalFrame = new Mat();
        //    Mat homography = new Mat();
        //    Mat Canny = new Mat();
        //    int FrameHeigh = 270;

        //    PointF[] PointGoal = new PointF[4]
        //        {
        //        new PointF(PointP[0].X, PointP[0].Y-FrameHeigh),
        //        new PointF(PointP[1].X, PointP[1].Y-FrameHeigh),
        //        new PointF(PointP[0].X, PointP[0].Y-200),
        //        new PointF(PointP[1].X, PointP[1].Y-200)
        //        };
        //    PointF[] PointGoalCropTo = new PointF[4]
        //         {
        //          new PointF(0, 0),
        //          new PointF(1920, 0),
        //          new PointF(0,FrameHeigh ),
        //          new PointF(1920,FrameHeigh)
        //         };

        //    CvInvoke.FindHomography(PointGoal, PointGoalCropTo, homography, HomographyMethod.Ransac);
        //    CvInvoke.WarpPerspective(frame, GoalFrame, homography, new Size(1920, 1080));            


        //    CvInvoke.Canny(GoalFrame, Canny, Vupper, Vlower,5,true);            

        //    List<Triangle2DF> triangleList = new List<Triangle2DF>();
        //    List<RotatedRect> boxList = new List<RotatedRect>(); //a box is a rotated rectangle

        //    Mat hierachy = new Mat();
        //    VectorOfVectorOfPoint contours = new VectorOfVectorOfPoint();
        //    CvInvoke.FindContours(Canny, contours, hierachy, RetrType.List, ChainApproxMethod.ChainApproxSimple);


        //    #region each contour Operation

        //    int count = contours.Size;

        //    for (int i = 0; i < count; i++)
        //    {               
        //        VectorOfPoint approxContour = new VectorOfPoint();
        //        CvInvoke.ApproxPolyDP(contours[i], approxContour, CvInvoke.ArcLength(contours[i], true) * 0.04, true);
        //        double a = CvInvoke.ContourArea(contours[i], false);  //  Find the area of contour                        

                

        //            #region Extract Data from contour

        //            Point[] pts = approxContour.ToArray();  // Point angle

        //            //Calc center Point
        //            MCvMoments m = new MCvMoments();
        //            m = CvInvoke.Moments(contours[i]);
        //            double cx = m.M10 / m.M00;
        //            double cy = m.M01 / m.M00;
        //            double realCx = (1920 - cx) / Xdivider;
        //            double realCy = cy / (FrameHeigh/111.0);


        //            //Orientation Method 1 using min area rectangle
        //            RotatedRect rRect = CvInvoke.MinAreaRect(approxContour);                    

        //            #endregion


        //        CvInvoke.Circle(GoalFrame, new Point((int)cx, (int)cy), 5, new MCvScalar(255, 255, 0), -1);
        //        CvInvoke.DrawContours(GoalFrame, contours, i, new MCvScalar(0, 0, 255), 3);

        //        #region Triangle

        //        if (approxContour.Size == 3)
        //            {
                        


        //            }

        //            #endregion

        //            #region Rectangle 

        //            if (approxContour.Size == 4)
        //            {
        //                float ratio = rRect.Size.Width / rRect.Size.Height;
                                            
        //                // Square
        //                if (ratio > 0.6 && ratio < 1.4)
        //                {
        //                }
        //                // Rectangle
        //                else
        //                {
        //                }

        //            }
        //            #endregion

        //            #region Circle

        //            if (approxContour.Size > 4)
        //            {
        //            }
        //            #endregion
        //   }



                


        //    #endregion

        //    CvInvoke.Imshow("GoalFrame", Canny);

        //}

    
    


    }
}
