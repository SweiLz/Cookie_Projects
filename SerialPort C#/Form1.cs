using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace SerialPortSimple
{
    public partial class Form1 : Form
    {

        bool spFlag = false;

        string PortName = "COM3";

        const int DISPLAY_POINTS = 20;

        SerialPort Port;

        List<int> adc0 = new List<int>();
        List<int> adc1 = new List<int>();
        List<int> adc2 = new List<int>();
        List<int> adc3 = new List<int>();

        List<int> xAxis = new List<int>();

        int PointCount = 0;
        int PointStart = 0;

        public Form1()
        {
            InitializeComponent();
        }

        private void Form1_Load(object sender, EventArgs e)
        {
          
        }

      


        private void Port_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            //string line = Port.ReadLine();
            //line = line.Replace(":", "").Replace(" ", "").Replace("\r", "").Replace("\n", "");
            //string[] adcs = line.Split(new string[] { "," }, StringSplitOptions.None);
            //Invoke((MethodInvoker)(() => PlotADC(adcs)));

            try
            {
                string line = Port.ReadExisting();

                richTextBox1.AppendText("<  COM   >    " + line + "\n");  //Add value to richTextBox         
                
            }
            catch { }
        }

        private void Form1_FormClosed(object sender, FormClosedEventArgs e)
        {
            if(spFlag)
            {
                Port.Close();
            }
        }

        private void ComSelect_DropDown(object sender, EventArgs e)
        {
            ComSelect.Items.Clear();
            string[] ports = SerialPort.GetPortNames();
            foreach (string comport in ports)
            {
                ComSelect.Items.Add(comport);
            }
        }

        private void ConnectCom_Click(object sender, EventArgs e)
        {
            if (spFlag == false)
            {
                try
                {
                    Port = new SerialPort(ComSelect.Text, Int32.Parse(comboBox1.Text) , Parity.None, 8, StopBits.One);


                    Port.DataReceived += new SerialDataReceivedEventHandler(Port_DataReceived);
                    Port.Open();
                    if (Port.IsOpen)
                    {
                        ConnectCom.BackColor = Color.LightGreen;
                        ConnectCom.Text = "Disconnect";
                        spFlag = true;
                        button1.Enabled = true;
                        
                    }
                }
                catch { }
            }
            else
            {
                Port.Close();
                ConnectCom.BackColor = Color.WhiteSmoke;
                ConnectCom.Text = "Connect";
                spFlag = false;
                button1.Enabled = false;
            }
        }




 
        private void button1_Click(object sender, EventArgs e)
        {
            string Buffer = textBox1.Text + textBox2.Text + textBox3.Text+textBox4.Text;
            Port.Write(Buffer);
            richTextBox1.AppendText("<  SEND  >    " + Buffer+"\n");
        }

        private void label5_Click(object sender, EventArgs e)
        {

        }

        private void button2_Click(object sender, EventArgs e)
        {
            richTextBox1.Clear();
        }
    }
}
