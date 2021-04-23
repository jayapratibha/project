using System;                              //these are the header files to display
using System.Collections.Generic;
using System.ComponentModel;               //controls
using System.Data;
using System.Drawing;                      // rectangular box
using System.Linq;                         // extrsct ROI or eye crop
using System.Text;                         // msg on bottom
using System.Threading;                    //parallel processing
using System.Windows.Forms;                // forms to user



using OpenCvSharp;                         //we are using csharp and open cv

namespace OpenCVWinForm                    //entire work is in a form
{
    public partial class MainForm : Form
    {
        private Thread _cameraThread;

        public MainForm()
        {
            InitializeComponent();
        }

        #region Camera Thread
        private void CaptureCamera()        //capture the video by creating a thread
        {
            _cameraThread = new Thread(new ThreadStart(CaptureCameraCallback));    //calls capturecameracallback
            _cameraThread.Start();
        }

        private void CaptureCameraCallback()
        {
            const double ScaleFactor = 2.5;   //as scalefactor increases image size increases or zoom
            const int MinNeighbors = 1;             //checks whether nearby pixel is related and forms an eye
            CvSize MinSize = new CvSize(30, 30);   //rectangle size

            CvCapture cap = CvCapture.FromCamera(1);
            CvHaarClassifierCascade cascade = CvHaarClassifierCascade.FromFile("haarcascade_eye.xml"); //here eye pattern
            while (true)                                                                               //is given
            {
                IplImage img = cap.QueryFrame();       //retrieves frames
                CvSeq<CvAvgComp> eyes = Cv.HaarDetectObjects(img, cascade, Cv.CreateMemStorage(), ScaleFactor, MinNeighbors, HaarDetectionType.DoCannyPruning, MinSize);
                                                              //identify ROI for each
                Bitmap left1 = BitmapConverter.ToBitmap(img);  //we are converting image to bitmap(direct)
                Bitmap left2 = BitmapConverter.ToBitmap(img);  //take as flipped image
                Bitmap right1 = BitmapConverter.ToBitmap(img);  //take as direct
                Bitmap right2 = BitmapConverter.ToBitmap(img);  //take as flipped image

                int left = 0;
                int right = 0;

                foreach (CvAvgComp eye in eyes.AsParallel())
                {
                    img.DrawRect(eye.Rect, CvColor.Blue);         //for each eye draw a blue rectangle

                    if (eye.Rect.Left > pctCvWindow.Width / 2)    //whichever eye is nearer to the window
                    {
                        try
                        {
                            IplImage rightEyeImg1 = img.Clone();
                            Cv.SetImageROI(rightEyeImg1, eye.Rect);
                            IplImage rightEyeImg2 = Cv.CreateImage(eye.Rect.Size, rightEyeImg1.Depth, rightEyeImg1.NChannels);
                            Cv.Copy(rightEyeImg1, rightEyeImg2, null);
                            Cv.ResetImageROI(rightEyeImg1);

                            
                            Bitmap rightEyeBm = BitmapConverter.ToBitmap(rightEyeImg2);
                            Bitmap actual = BitmapConverter.ToBitmap(rightEyeImg2);
                            rightEyeBm.RotateFlip(RotateFlipType.Rotate180FlipY);  //take right eye clone and flip
                            left1 = rightEyeBm;
                            right1 = actual;
                            right = 1;
                        }
                        catch { }
                    }
                    else
                    {
                        try
                        {
                            IplImage leftEyeImg1 = img.Clone();
                            Cv.SetImageROI(leftEyeImg1, eye.Rect);
                            IplImage leftEyeImg2 = Cv.CreateImage(eye.Rect.Size, leftEyeImg1.Depth, leftEyeImg1.NChannels);
                            Cv.Copy(leftEyeImg1, leftEyeImg2, null);
                            Cv.ResetImageROI(leftEyeImg1);

                            Bitmap leftEyeBm = BitmapConverter.ToBitmap(leftEyeImg2);
                            Bitmap actual = BitmapConverter.ToBitmap(leftEyeImg2);
                            leftEyeBm.RotateFlip(RotateFlipType.Rotate180FlipY);  //take left eye clone and flip it
                            left2 = actual;
                            right2 = leftEyeBm;
                            left = 1;
                        }
                        catch { }
                    }
                }

                if (left == 1 && right == 1)            //display both
                {
                    pctRightEye.Image = right1;
                    pctLeftEye.Image = left2;
                }
                else
                {
                    if (left == 1)                      //display only left
                    {
                        pctRightEye.Image = right2;
                        pctLeftEye.Image = left2;
                    }
                    else                                //display only right
                    {
                        pctRightEye.Image = right1;
                        pctLeftEye.Image = left1;
                    }
                }


                Bitmap bm = BitmapConverter.ToBitmap(img);
                bm.SetResolution(pctCvWindow.Width, pctCvWindow.Height);
                pctCvWindow.Image = bm;

                img = null;
                bm = null;
            }
        }
        #endregion

        #region Form Handlers
        private void MainForm_FormClosing(object sender, FormClosingEventArgs e)
        {
            if (_cameraThread != null && _cameraThread.IsAlive)
            {
                _cameraThread.Abort();                            //for stop
            }
        }
        #endregion

        #region Button Handlers
        private void btnStart_Click(object sender, EventArgs e)
        {
            if (btnStart.Text.Equals("Start"))
            {
                CaptureCamera();
                btnStart.Text = "Stop";
            }
            else
            {
                _cameraThread.Abort();
                btnStart.Text = "Start";
            }
        }

        private void btnSaveImage_Click(object sender, EventArgs e)
        {
            if (saveFileDialog1.ShowDialog() == DialogResult.OK)
            {
                String imageFileName = saveFileDialog1.FileName;       //save the image
                pctCvWindow.Image.Save(imageFileName);
            }
        }
        #endregion
    }
    public partial class Form1 : Form
    {
        Process p;
        [DllImport("User32")]                   //access camera
        private static extern int ShowWindow(int hwnd, int nCmdShow);
        [DllImport("user32.dll")]               //access mouse
        static extern IntPtr SetParent(IntPtr hWndChild, IntPtr hWndNewParent); 


        private const int SW_HIDE = 0; 
        public Form1()
        {
            InitializeComponent();
        }
                                                                //pattern detection on clicking executes 
        private void button1_Click(object sender, EventArgs e)  //button1_Click on clicking start this gets activated
        {
			double	[]movement_dist = new double[movement_count];
			double	[]movement_angle = new double[movement_count];
                                                                   //var for dist,angle,loc co-ordinates
			double	[]movement_x = new double[movement_count];
			double	[]movement_y = new double[movement_count];
			int frame_count = 0;
			for(;;)                                                //infinite for loop
			{
				frame_count++;
				if(is_tracking())
				{
					movement_x[i] = get_location_x(frame_count);   //get x and y co-ordinates from frame
					movement_y[i] = get_location_y(frame_count);
				}
				else
					break;
                                             // i= current frame,i-1 = prev frame the diff betw them gives movement
				if(frame_count > 1)
				{
					double xdiff = movement_x[i]-movement_x[i-1];
					double ydiff = movement_y[i]-movement_y[i-1];
					movement_dist[i] = Math.sqrt(Math.pow(movement_x[i]-movement_x[i-1],2)+Math.pow(movement_y[i]-movement_y[i-1],2));
					movement_angle[i] = Math.arctan2(ydiff,xdiff); we are calc eucledian dist and theta
					
					
					
					
					double[][] W = null;   //var for weight
					double[] bias = null;

					ANN ann = new ANN(frame_count,movement_dist,movement_angle,W,bias);
					double[][] activationBias = new double[S_FEATURE];
		
					ann.formNeural(movement_dist,movement_angle,MAX_LAYER);        //forms neural network
					ann.pattern_mapping(ann.bias,activationBias);   //we get values from neural network as 0 and 1 act fn
                                                                    
					double	[]MouseCoordinates = new double[activationBias.length];
					for(int i=0;i<activationBias.length;i++)
					{
						MouseCoordinates[i] = activationBias[i];   //we convert act fn to mouse co-ordinates
					}
			
					MouseLnk(MouseCoordinates);  //here

					int hWnd;
					p = new Process();
					p.StartInfo.FileName = "Mouse.lnk";
					p.Start();
					hWnd = p.MainWindowHandle.ToInt32();   // hwnd is parent window toint 32 we access mouse harware here
					//ShowWindow(hWnd, SW_HIDE);

					p.WaitForInputIdle();                 //when we are blinking or clicking
					while (p.MainWindowHandle == IntPtr.Zero)  //come to middle of screen
					{
						p.Refresh ();                         //continuously check
					}
					SetParent(p.MainWindowHandle, this.panel1.Handle);				
				}
			}
        }

        private void button2_Click(object sender, EventArgs e)
        {
            p.Kill();
            p.Close();                        //on clicking anything else then process terminates
        }
    }
    public class ANN {
        public var N;
        public var n_output;
		public var n_hidden;
		public double[][] W;
		public double[] hbias;
		public double[] vbias;
		public Random rng;

        public void display()
        {
            Console.println("W");
            for(int i=0;i<n_hidden;i++)
            {
                for(int j=0;j<n_output;j++)
                {
                    Console.print(W[i][j] + " ");
                }
                Console.println();
            }
            Console.println("HBias");
            for(int i=0;i<n_hidden;i++)
            {
                Console.print(hbias[i] + " ");
            }
            Console.println();
            Console.println("VBias");
            for(int i=0;i<n_output;i++)
            {
                Console.print(vbias[i] + " ");
            }
            Console.println();
        }

	public double uniform(double min, double max) {
            return rng.nextDouble() * (max - min) + min;
	}

	public var constructive(var n, double p) {
            if(p < 0 || p > 1) return 0;   //if condition stop
            
            var c = 0;
            double r;
            
            for(int i=0; i<n; i++)	{
                r = rng.nextDouble();     //else keep gen random nos
                if (r < p) c++;
            }
            return c;
        }

	public double sigmoid(double x) {            //sigmoid formula
            return 1.0 / (1.0 + Math.pow(Math.E, -x));
        }

	public ANN(var N,    var n_output, var n_hidden, double[][] W, double[] hbias, double[] vbias) {
            this.N = N;
            this.n_output = n_output;
            this.n_hidden = n_hidden;
            
            this.rng = new Random();
            
            if(W == null) {
                this.W = new double[this.n_hidden][this.n_output];
                double a = 1.0 / this.n_output;
                for(int i=0; i<this.n_hidden; i++) {
                    for(int j=0; j<this.n_output; j++) {
                        this.W[i][j] = uniform(-a, a);
                    }
                }
            } else {
                this.W = W;
            }

            if(hbias == null) {
                this.hbias = new double[this.n_hidden];
                for(int i=0; i<this.n_hidden; i++) this.hbias[i] = 0;
            } else {
                this.hbias = hbias;
            }

            if(vbias == null) {
                this.vbias = new double[this.n_output];
                for(int i=0; i<this.n_output; i++) this.vbias[i] = 0;
            } else {
                this.vbias = vbias;
            }
        }        //assign hbias and vbias

        public void formNeural(double[] input, double entropy, var k) {
            double[] ph_mean = new double[n_hidden];  //prev hidden we calc average
            double[] ph_sample = new double[n_hidden]; //for next 4 co-or we choose next 2 samples
            double[] nv_means = new double[n_output];  //next vertex
            double[] nv_samples = new double[n_output];
            double[] nh_means = new double[n_hidden];
            double[] nh_samples = new double[n_hidden];

            sample_h_given_v(input, ph_mean, ph_sample);

            for(int step=0; step<k; step++) {
                if(step == 0) {           //prev layer output to current layer input
                    apply_entropy(ph_sample, nv_means, nv_samples, nh_means, nh_samples);
                } else {                  //current layer output to next layer input
                    apply_entropy(nh_samples, nv_means, nv_samples, nh_means, nh_samples);
                }
            }

            for(int i=0; i<n_hidden; i++) {
                for(int j=0; j<n_output; j++) {
                    W[i][j] += entropy *(ph_mean[i] * input[j] - nh_means[i] * nv_samples[j]) / N;  //ANN formula
                }
                hbias[i] += entropy * (ph_sample[i] - nh_means[i]) / N;  //creates hidden layer activation fn
            }

            for(int i=0; i<n_output; i++) {
                vbias[i] += entropy * (input[i] - nv_samples[i]) / N;    //creates vertex activation fn
            }
	}       

	public void sample_h_given_v(double[] v0_sample, double[] mean, double[] sample) {
            for(int i=0; i<n_hidden; i++) {
                mean[i] = probDisribution(v0_sample, W[i], hbias[i]);
                sample[i] = constructive(1, mean[i]);
            }
        }

        public void sample_v_given_h(double[] h0_sample, double[] mean, double[] sample) {
            for(int i=0; i<n_output; i++) {
                mean[i] = probVectorization(h0_sample, i, vbias[i]);  //here we form the final output from intermediate
                sample[i] = constructive(1, mean[i]);                 //hidden layer
            }
        }

        public double probDisribution(double[] v, double[] w, double b) {
            double pre_sigmoid_activation = 0.0;
            for(int j=0; j<n_output; j++) {
                pre_sigmoid_activation += w[j] * v[j];
            }
            pre_sigmoid_activation += b;
            return sigmoid(pre_sigmoid_activation);
        }

        public double probVectorization(double[] h, var i, double b) {
            double pre_sigmoid_activation = 0.0;
            for(int j=0; j<n_hidden; j++) {
                pre_sigmoid_activation += W[j][i] * h[j];
            }
            pre_sigmoid_activation += b;
            return sigmoid(pre_sigmoid_activation);
        }
        
        public void apply_entropy(double[] h0_sample, double[] nv_means, double[] nv_samples, double[] nh_means, double[] nh_samples) {
            sample_v_given_h(h0_sample, nv_means, nv_samples);
            sample_h_given_v(nv_samples, nh_means, nh_samples);
        }

        public void pattern_mapping(double[] v, double[] reconstructed_v) {
            double[] h = new double[n_hidden];
            double pre_sigmoid_activation;

            for(int i=0; i<n_hidden; i++) {
                h[i] = probDisribution(v, W[i], hbias[i]);    //v is hidden polar co-ordinates
            }

            for(int i=0; i<n_output; i++) {
                pre_sigmoid_activation = 0.0;          //before applying activation

                for(int j=0; j<n_hidden; j++) {
                    pre_sigmoid_activation += W[j][i] * h[j];
                }

                pre_sigmoid_activation += vbias[i];
                reconstructed_v[i] = sigmoid(pre_sigmoid_activation);
            }
        }
	}
}