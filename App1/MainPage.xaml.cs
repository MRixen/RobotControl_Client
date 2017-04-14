﻿using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices.WindowsRuntime;
using System.Threading;
using Windows.Devices.I2c;
using Windows.Devices.Spi;
using Windows.Foundation;
using Windows.Foundation.Collections;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using Windows.UI.Xaml.Data;
using Windows.UI.Xaml.Input;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Navigation;
using Windows.Devices.Enumeration;
using Windows.Devices.Gpio;
using System.Diagnostics;
using Windows.System.Threading;
using Windows.ApplicationModel.Background;
using Windows.Storage;
using Windows.Storage.Search;
using System.Threading.Tasks;
using CanTest;
using Windows.UI;

// The Blank Page item template is documented at http://go.microsoft.com/fwlink/?LinkId=402352&clcid=0x409

namespace App1
{
    public sealed partial class MainPage : Page
    {
        // TODO: Add something to shutdown everything (with stopAllOperations flag)
        // TODO: Change sensor data to acquire only the constant g values -> low pass filter
        private MCP2515 mcp2515;
        private Timer stateTimer, errorTimer;
        private const byte SPI_CHIP_SELECT_LINE = 0;
        private int DELTA_T_TIMER_CALLBACK = 5, DELTA_T_MCP_EXECUTOR = 100, DELTA_T_ERROR_TIMER = 10;
        private const double ENCODER_TO_DEGREE = ((double)360 / 34608);

        // DATA FOR ADXL SENSOR
        private const byte ACCEL_REG_X = 0x32;              /* Address of the X Axis data register                  */
        private const byte ACCEL_REG_Y = 0x34;              /* Address of the Y Axis data register                  */
        private const byte ACCEL_REG_Z = 0x36;              /* Address of the Z Axis data register                  */
        private const byte ACCEL_I2C_ADDR = 0x53;           /* 7-bit I2C address of the ADXL345 with SDO pulled low */
        private const byte ACCEL_SPI_RW_BIT = 0x80;         /* Bit used in SPI transactions to indicate read/write  */
        private const byte ACCEL_SPI_MB_BIT = 0x40;         /* Bit used to indicate multi-byte SPI transactions     */
        private const int ACCEL_RES = 1024;         /* The ADXL345 has 10 bit resolution giving 1024 unique values                     */
        private const int ACCEL_DYN_RANGE_G = 8;    /* The ADXL345 had a total dynamic range of 8G, since we're configuring it to +-4G */
        private const int UNITS_PER_G = ACCEL_RES / ACCEL_DYN_RANGE_G;  // Ratio of raw int values to G units          

        // CONTENT
        // 0. byte: Number of the task (if no task is selected this value is 0 > Robot shall move, e.g. no tasks)
        // 1. byte: ID of the motor to move
        // 2-3. byte: Velocity to move the motor to the specified position
        // 4-5. byte: Endposition for the specific motor 
        // 6. byte: Direction of the motor
        // 7. byte: 
        private byte[] bytesToSend = new byte[8];

        struct McpExecutorDataFrame
        {
            public double pwmValue;
            public double encoderValue;
            public int ident;
            public float timeStamp;
        };

        // Data to set the execution context for checking the message answer from remote mcpExecutor when sending stop / start sequence
        enum CheckExecution
        {
            stopExecution,
            startExecution
        };

        enum HmiElementsStates
        {
            disableAll,
            enableAll,
            startIsPressed,
            stoppIsPressed,
            checkBoxIsSelected
        };

        private GlobalDataSet globalDataSet;
        private ServerUnit serverComm;
        private ClientUnit clientUnit;
        private Diagnose diagnose;
        private Task task_mcpExecutorService;
        private HmiElementsStates hmiElementsStates;

        private Pulses pulses;
        private const byte MAX_MCP_DEVICE_COUNTER = 2; // max. 255 
        private int MAX_WAIT_TIME = 5000; // milliseconds 
        private GpioPin[] mcpExecutor_request = new GpioPin[MAX_MCP_DEVICE_COUNTER];
        private GpioPin[] mcpExecutor_handshake = new GpioPin[MAX_MCP_DEVICE_COUNTER];
        private int mcpExecutorCounter;

        // DATA FOR ERROR HANDLING
        private const int MAX_ERROR_COUNTER_TRANSFER = 20;
        private int errorCounterTransfer;

        // DATA FOR DEBUGGING
        private Stopwatch stopwatch_timestamp = new Stopwatch();
        private Stopwatch timer_programExecution = new Stopwatch();
        private Stopwatch stopwatch_maxWaitMsgIn = new Stopwatch();
        private Stopwatch stopwatch_delay = new Stopwatch();
        private bool firstStart;
        private bool startSequenceIsActive = false;
        private bool stopSequenceIsActive;
        private bool getProgramDuration;
        private long timerValue;
        private long[] timerArray = new long[10];
        private double currentMotorAngle = 0;

        public MainPage()
        {
            this.InitializeComponent();

            // Initilize data
            errorCounterTransfer = 0;
            mcpExecutorCounter = 0;
            firstStart = true;
            stopSequenceIsActive = false;
            globalDataSet = new GlobalDataSet(); // Get things like mcp2515, logic_Mcp2515_Sender, logic_Mcp2515_Receiver
            //serverComm = new ServerUnit(globalDataSet);
            clientUnit = new ClientUnit(globalDataSet);
            diagnose = new Diagnose(globalDataSet);
            mcp2515 = globalDataSet.Mcp2515;
            pulses = new Pulses();


            // USER CONFIGURATION
            globalDataSet.DebugMode = false;
            getProgramDuration = false;

            // Inititalize raspberry pi and gpio
            init_raspberry_pi_gpio();
            init_raspberry_pi_spi();

            // Inititalize mcp2515
            Task task_initMcp2515 = new Task(globalDataSet.init_mcp2515_task);
            task_initMcp2515.Start();
            task_initMcp2515.Wait();

            // Start executor service
            task_mcpExecutorService = new Task(mcpExecutorService_task);
            task_mcpExecutorService.Start();

            // Inititalize background tasks
            stateTimer = new Timer(this.StateTimer, null, 0, DELTA_T_TIMER_CALLBACK); // Create timer to display the state of message transmission
            errorTimer = new Timer(this.ErrorTimer, null, 0, DELTA_T_ERROR_TIMER); // Create timer to display the state of message transmission

            // Inititalize server
            //Task<bool> serverStarted = serverComm.StartServer();
            clientUnit.StartClient_loop();
        }

        private void init_raspberry_pi_gpio()
        {
            if (globalDataSet.DebugMode) Debug.Write("Start GPIO init \n");

            var gpioController = GpioController.GetDefault();

            if (gpioController == null)
            {
                return;
            }
            try
            {
                if (globalDataSet.DebugMode) Debug.Write("Configure pins \n");
                // Configure pins
                globalDataSet.do_mcp2515_cs_sen = configureGpio(gpioController, (int)RASPBERRYPI.GPIO.GPIO_19, GpioPinValue.High, GpioPinDriveMode.Output);
                globalDataSet.di_mcp2515_int_sen = configureGpio(gpioController, (int)RASPBERRYPI.GPIO.GPIO_5, GpioPinDriveMode.Input);
                globalDataSet.do_mcp2515_cs_rec = configureGpio(gpioController, (int)RASPBERRYPI.GPIO.GPIO_12, GpioPinValue.High, GpioPinDriveMode.Output);
                globalDataSet.di_mcp2515_int_rec = configureGpio(gpioController, (int)RASPBERRYPI.GPIO.GPIO_13, GpioPinDriveMode.Input);
                globalDataSet.do_startAcquisition = configureGpio(gpioController, (int)RASPBERRYPI.GPIO.GPIO_17, GpioPinValue.Low, GpioPinDriveMode.Output);
                globalDataSet.do_saveActEncPos = configureGpio(gpioController, (int)RASPBERRYPI.GPIO.GPIO_21, GpioPinValue.High, GpioPinDriveMode.Output);
            }
            catch (FileLoadException ex)
            {
                if (globalDataSet.DebugMode) Debug.Write("Exception in initGPIO: " + ex + "\n");
            }
        }

        private async void init_raspberry_pi_spi()
        {
            if (globalDataSet.DebugMode) Debug.Write("Init SPI interface" + "\n");
            try
            {
                var settings = new SpiConnectionSettings(SPI_CHIP_SELECT_LINE);
                settings.ClockFrequency = 5000000;
                settings.Mode = SpiMode.Mode3;
                string aqs = SpiDevice.GetDeviceSelector();
                var dis = await DeviceInformation.FindAllAsync(aqs);
                globalDataSet.SPIDEVICE = await SpiDevice.FromIdAsync(dis[0].Id, settings);
                if (globalDataSet.SPIDEVICE == null)
                {
                    if (globalDataSet.DebugMode) Debug.Write("SPI Controller is currently in use by another application. \n");
                    return;
                }
            }
            catch (Exception ex)
            {
                if (globalDataSet.DebugMode) Debug.Write("SPI Initialization failed. Exception: " + ex.Message + "\n");
                return;
            }

            // Send something to check that spi device is ready
            //globalDataSet.Spi_not_initialized = true;
            while (globalDataSet.Spi_not_initialized)
            {
                bool error = false;
                try
                {
                    globalDataSet.SPIDEVICE.Write(new byte[] { 0xFF });
                }
                catch (Exception)
                {
                    error = true;
                }
                if (!error)
                {
                    globalDataSet.Spi_not_initialized = false;
                    if (globalDataSet.DebugMode) Debug.Write("Spi device ready" + "\n");
                }
                else
                {
                    if (globalDataSet.DebugMode) Debug.Write("Spi device not ready" + "\n");
                }
            }
        }

        private GpioPin configureGpio(GpioController gpioController, int gpioId, GpioPinDriveMode pinDriveMode)
        {
            GpioPin pinTemp;

            pinTemp = gpioController.OpenPin(gpioId);
            pinTemp.SetDriveMode(pinDriveMode);

            return pinTemp;
        }

        private GpioPin configureGpio(GpioController gpioController, int gpioId, GpioPinValue pinValue, GpioPinDriveMode pinDriveMode)
        {
            GpioPin pinTemp;

            pinTemp = gpioController.OpenPin(gpioId);
            pinTemp.Write(pinValue);
            pinTemp.SetDriveMode(pinDriveMode);

            return pinTemp;
        }

        private void StateTimer(object state)
        {
            bool indicatorMode = false;

            if (globalDataSet.di_mcp2515_int_rec.Read() == GpioPinValue.Low)
            {
                indicatorMode = true;
            }
            else
            {
                indicatorMode = false;
            }

            /* UI updates must be invoked on the UI thread */
            var task = this.Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, () =>
                {
                    if (indicatorMode)
                    {
                        indicator.Background = new SolidColorBrush(Colors.Green);
                    }
                    else
                    {
                        indicator.Background = new SolidColorBrush(Colors.Red);
                    }

                });
        }

        private void ErrorTimer(object state)
        {
            // TODO Show red blinking warning message on screen
            if (errorCounterTransfer >= MAX_ERROR_COUNTER_TRANSFER)
            {
                if (globalDataSet.DebugMode) Debug.Write("ERROR TRANSFER - STOP ALL OPERATIONS" + "\n");
                globalDataSet.StopAllOperations = true;
                errorCounterTransfer = 0;
            }
        }

        private byte[] generateIdentifier(int identifierTemp)
        {
            byte[] identifier = BitConverter.GetBytes(identifierTemp);
            if (BitConverter.IsLittleEndian)
            {
                Array.Reverse(identifier);
                if (globalDataSet.DebugMode) Debug.Write("IsLittleEndian \n");
            }

            if (globalDataSet.DebugMode) Debug.Write("Convert " + identifierTemp + " to " + identifier[0] + " and " + identifier[1] + "\n");
            if (globalDataSet.DebugMode) Debug.Write("Convert " + identifierTemp + " to " + identifier + "\n");

            // Return max 2 bytes
            return identifier;
        }

        private void SendMotorData(byte rxStateIst, byte rxStateSoll)
        {
            byte[] returnMessage = new byte[mcp2515.MsgSizeFromMcp];
            byte[] returnMessageTemp = new byte[1];

            if ((rxStateIst & rxStateSoll) == 1)
            {
                byte[] spiMessage = new byte[1];

                //for (int i = 0; i < mcp2515.MessageSizeAdxl; i++) returnMessage[i] = globalDataSet.LOGIC_MCP2515_RECEIVER.mcp2515_read_buffer(mcp2515.REGISTER_RXB0Dx[i]);
                returnMessage = globalDataSet.LOGIC_MCP2515_RECEIVER.mcp2515_read_buffer_v3(mcp2515.SPI_INSTRUCTION_READ_RX_BUFFER0);
                // We need to check sidl only because we have not so much devices.
                //Debug.WriteLine("REGISTER_RXB0SIDL: " + globalDataSet.LOGIC_MCP2515_RECEIVER.mcp2515_read_buffer(mcp2515.REGISTER_RXB0SIDL));
            }
            else if ((rxStateIst & rxStateSoll) == 2)
            {
                //for (int i = 0; i < mcp2515.MessageSizeAdxl; i++) returnMessage[i] = globalDataSet.LOGIC_MCP2515_RECEIVER.mcp2515_read_buffer(mcp2515.REGISTER_RXB1Dx[i]);
                returnMessage = globalDataSet.LOGIC_MCP2515_RECEIVER.mcp2515_read_buffer_v3(mcp2515.SPI_INSTRUCTION_READ_RX_BUFFER1);
                //Debug.WriteLine("REGISTER_RXB1SIDL: " + globalDataSet.LOGIC_MCP2515_RECEIVER.mcp2515_read_buffer(mcp2515.REGISTER_RXB1SIDL));
            }
        }

        private void MainPage_Unloaded(object sender, object args)
        {
            globalDataSet.SPIDEVICE.Dispose();
        }

        private void button_start_Click(object sender, RoutedEventArgs e)
        {
            startSequenceIsActive = true;
            changeHmiElements(HmiElementsStates.startIsPressed);
            currentMotorAngle = slider.Value;
        }

        private void button_stopp_Click(object sender, RoutedEventArgs e)
        {
            startSequenceIsActive = false;
            changeHmiElements(HmiElementsStates.stoppIsPressed);
        }

        public async void mcpExecutorService_task()
        {
            await Task.Run(() => controlLoop());
        }

        private void checkBox_sinus_Checked(object sender, RoutedEventArgs e)
        {
            pulses.active_pulse_type = Pulses.Pulse_Types.sinus;
            changeHmiElements(HmiElementsStates.checkBoxIsSelected);
        }

        private void checkBox_sawtooth_Checked(object sender, RoutedEventArgs e)
        {
            pulses.active_pulse_type = Pulses.Pulse_Types.sawtooth;
            changeHmiElements(HmiElementsStates.checkBoxIsSelected);
        }

        private void checkBox_square_Checked(object sender, RoutedEventArgs e)
        {
            pulses.active_pulse_type = Pulses.Pulse_Types.square;
            changeHmiElements(HmiElementsStates.checkBoxIsSelected);
        }

        private void changeHmiElements(HmiElementsStates state)
        {
            // Change hmi elements
            switch (state)
            {
                case HmiElementsStates.disableAll:
                    slider.IsEnabled = false;
                    button_stopp.IsEnabled = false;
                    button_start.IsEnabled = false;
                    break;
                case HmiElementsStates.enableAll:
                    slider.IsEnabled = true;
                    button_stopp.IsEnabled = true;
                    button_start.IsEnabled = true;
                    break;
                case HmiElementsStates.startIsPressed:
                    button_stopp.IsEnabled = true;
                    button_start.IsEnabled = false;
                    break;
                case HmiElementsStates.stoppIsPressed:
                    button_stopp.IsEnabled = false;
                    button_start.IsEnabled = true;
                    break;
                case HmiElementsStates.checkBoxIsSelected:
                    break;
                default:
                    break;
            }
        }

        private void changeActivePulse(Pulses.Pulse_Types pulseType)
        {
            switch (pulseType)
            {
                case Pulses.Pulse_Types.sinus:
                    break;
                case Pulses.Pulse_Types.square:
                    break;
                case Pulses.Pulse_Types.sawtooth:
                    break;
                case Pulses.Pulse_Types.counter:
                    break;
                default:
                    break;
            }
            changeHmiElements(HmiElementsStates.checkBoxIsSelected);
            pulses.active_pulse_type = pulseType;
        }

        private int[] getActivePulseData()
        {
            switch (pulses.active_pulse_type)
            {
                case Pulses.Pulse_Types.sinus:
                    return pulses.Pulse_sinus;
                case Pulses.Pulse_Types.square:
                    return pulses.Pulse_square;
                case Pulses.Pulse_Types.sawtooth:
                    return pulses.Pulse_sawtooth;
                case Pulses.Pulse_Types.counter:
                    return pulses.Pulse_counter;
                default:
                    return pulses.Pulse_sinus;
            }
        }

        // DESCRIPTION
        // Send soll data to motors
        // Receive angle position from motors and set it to global variable
        private void controlLoop()
        {
            long startTimeCheck = 0;
            //bool preCondIsSet = false;
            if (!stopwatch_delay.IsRunning) stopwatch_delay.Restart();

            while (!globalDataSet.StopAllOperations)
            {
                // Wait until spi device is ready
                while (globalDataSet.Spi_not_initialized) if (globalDataSet.DebugMode) Debug.WriteLine("wait for spi device");

                // Send motor angle end position (that comes from server) to device via spi to can bus
                sendDataToShields();

                // Delay for program execution. Its neccessary to avoid failures in data transmission
                delay(startTimeCheck, 20);

                // Read encoder value from device via spi from can bus
                receiveDataFromShields();

                // Delay for program execution. Its neccessary to avoid failures in data transmission
                delay(startTimeCheck, 20);

            }
        }

        private void stopWatchStop(Stopwatch stopwatch)
        {
            if (stopwatch.IsRunning)
            {
                stopwatch.Stop();
                stopwatch.Reset();
            }
        }

        private void stopWatchStart(Stopwatch stopwatch)
        {
            if (!stopwatch.IsRunning)
            {
                stopwatch.Reset();
                stopwatch.Start();
            }
        }

        private void sendDataToClient(int pwm_value, double encoder_value, long time_stamp, int signal_id)
        {
            // Create string with sensor content
            string pwmValue = String.Format("x{0:F3}", pwm_value);
            string encoderValue = String.Format("y{0:F3}", encoder_value);
            string zText = String.Format("z{0:F3}", 1);
            string signal_Id = signal_id.ToString();
            string timeStamp = time_stamp.ToString();

            string message = pwmValue + "::" + encoderValue + "::" + zText + "::" + timeStamp;
            diagnose.sendToSocket(signal_Id, message);
        }

        private void slider_valueChanged(object sender, RangeBaseValueChangedEventArgs e)
        {
            currentMotorAngle = slider.Value;
            textBlock_istValue.Text = currentMotorAngle.ToString();
        }

        private void button_Click_saveActPos(object sender, RoutedEventArgs e)
        {
            if(globalDataSet.SaveActEncPos) globalDataSet.SaveActEncPos = false;
            else globalDataSet.SaveActEncPos = true;

            textBox_saveActPos_state.Text = globalDataSet.SaveActEncPos.ToString();
        }

        private void delay(long startTimeCHeck, long delayAmount)
        {
             // Wait some time
             startTimeCHeck = stopwatch_delay.ElapsedMilliseconds;
            while ((stopwatch_delay.ElapsedMilliseconds - startTimeCHeck) <= delayAmount) { }
        }

        private void sendDataToShields()
        {
            // Set motor direction byte and motor angle position bytes to byte array that is send to the motor driver
            //for (int i = 0; i < bytesToSend.Length; i++) bytesToSend[i] = globalDataSet.Incoming_DataPackage[i];
            bytesToSend = globalDataSet.Incoming_DataPackage;
            //if (bytesToSend[1] == 2)
            //{
            //    //Debug.WriteLine("returnMessage[1]: " + returnMessage[1]);
            //    Debug.WriteLine("action goes to motor 2: " + bytesToSend[0]);
            //}
            //for (int i = 0; i < bytesToSend.Length; i++) Debug.WriteLine("bytesToSend[" + i + "] " + bytesToSend[i]);

            // Send byte array to motor driver
            for (int j = 0; j < mcp2515.MessageSizeToMcp; j++) globalDataSet.LOGIC_MCP2515_RECEIVER.mcp2515_load_tx_buffer0(bytesToSend[j], j, mcp2515.MessageSizeToMcp);
            globalDataSet.LOGIC_MCP2515_RECEIVER.mcp2515_execute_rts_command(0);
        }

        private void receiveDataFromShields()
        {
            byte rxStateIst = 0x00;
            byte rxStateSoll = 0x03;

            // Wait until a message is received in buffer 0 or 1
            //stopwatch_maxWaitMsgIn.Reset();
            //stopwatch_maxWaitMsgIn.Start();
            //while ((globalDataSet.di_mcp2515_int_rec.Read() == GpioPinValue.High) && stopwatch_maxWaitMsgIn.ElapsedMilliseconds <= MAX_WAIT_TIME) { }
            if (globalDataSet.di_mcp2515_int_rec.Read() == GpioPinValue.Low)
            {
                //while ((globalDataSet.di_mcp2515_int_rec.Read() == GpioPinValue.High)) { }
                //stopwatch_maxWaitMsgIn.Stop();

                //if (stopwatch_maxWaitMsgIn.ElapsedMilliseconds < MAX_WAIT_TIME)
                //{
                // Check in which rx buffer the message is
                rxStateIst = globalDataSet.LOGIC_MCP2515_RECEIVER.mcp2515_get_state_command();

                if ((rxStateIst & rxStateSoll) == 1) globalDataSet.Outgoing_DataPackage = globalDataSet.LOGIC_MCP2515_RECEIVER.mcp2515_read_buffer_v3(mcp2515.SPI_INSTRUCTION_READ_RX_BUFFER0);
                else if ((rxStateIst & rxStateSoll) == 2) globalDataSet.Outgoing_DataPackage = globalDataSet.LOGIC_MCP2515_RECEIVER.mcp2515_read_buffer_v3(mcp2515.SPI_INSTRUCTION_READ_RX_BUFFER1);
                //}
            }

        }
    }
}


